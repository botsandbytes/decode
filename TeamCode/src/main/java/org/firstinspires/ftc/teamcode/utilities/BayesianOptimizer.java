package org.firstinspires.ftc.teamcode.utilities;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.ArrayRealVector;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Bayesian Optimizer using Gaussian Processes (GP) and Upper Confidence Bound (UCB).
 *
 * Optimizes a black-box function by building a probabilistic model (GP) of the objective
 * function and selecting the next point to evaluate by balancing exploration (high uncertainty)
 * and exploitation (low predicted error).
 */
public class BayesianOptimizer {

    private final Consumer<double[]> paramSetter;
    private final Supplier<Double> errorFunction;
    private final double[][] bounds; // [min, max] for each parameter
    private final int nParams;

    // GP Hyperparameters
    private final double lengthScale = 1.0; // Length scale for RBF kernel
    private final double signalVariance = 1.0; // Signal variance
    private final double noiseVariance = 1e-4; // Noise variance (assumed small for simulation/repeatable tests)
    private final double beta = 2.0; // Exploration weight for UCB

    // Data
    private final List<double[]> X_observed = new ArrayList<>();
    private final List<Double> y_observed = new ArrayList<>();

    private double[] bestParams;
    private double bestError = Double.MAX_VALUE;
    private int iteration = 0;
    private int initialSamples = 5; // Number of random samples before starting GP
    private int currentSample = 0;

    private final Random random = new Random();

    /**
     * @param bounds Min and max values for each parameter. dimensions: [nParams][2]
     * @param paramSetter Function to apply parameters
     * @param errorFunction Function to get error (minimize this)
     */
    public BayesianOptimizer(double[][] bounds, Consumer<double[]> paramSetter, Supplier<Double> errorFunction) {
        this.bounds = bounds;
        this.nParams = bounds.length;
        this.paramSetter = paramSetter;
        this.errorFunction = errorFunction;
        this.bestParams = new double[nParams];
    }

    /**
     * Call this method periodically.
     * @return true if optimization cycle is complete (this implementation runs indefinitely until stopped)
     */
    public boolean update() {
        // 1. Handle first call (Initialization)
        if (iteration == 0 && currentSample == 0) {
            double[] nextNorm;
            if (initialGuessNorm != null) {
                nextNorm = initialGuessNorm;
                initialGuessNorm = null; // Consume it
            } else {
                nextNorm = generateRandomParams();
            }

            // Apply denormalized params to robot
            paramSetter.accept(denormalize(nextNorm));
            // Store normalized params for next update() call
            storePendingParams(nextNorm);

            currentSample++;
            return false;
        }

        // 2. Collect result from previous run
        double error = errorFunction.get();
        double[] lastParamsNorm = getPendingParams(); // These are normalized

        X_observed.add(lastParamsNorm);
        y_observed.add(error);

        if (error < bestError) {
            bestError = error;
            bestParams = denormalize(lastParamsNorm); // Store real params
        }

        // 3. Select next parameters
        double[] nextNorm;

        if (currentSample < initialSamples) {
            // Random search phase
            nextNorm = generateRandomParams();
            currentSample++;
        } else {
            // Bayesian Optimization phase
            nextNorm = optimizeAcquisitionFunction();
            iteration++;
        }

        // 4. Apply and store
        paramSetter.accept(denormalize(nextNorm));
        storePendingParams(nextNorm);

        return false; // Never "finishes" automatically, user stops it
    }

    // Helper to store params between update calls
    private double[] pendingParams;
    private void storePendingParams(double[] p) {
        pendingParams = p;
    }
    private double[] getPendingParams() {
        return pendingParams;
    }

    private double[] generateRandomParams() {
        double[] p = new double[nParams];
        for (int i = 0; i < nParams; i++) {
            p[i] = random.nextDouble(); // 0.0 to 1.0 (Normalized)
        }
        return p;
    }

    /**
     * Maximizes the Acquisition Function (UCB) to find the next point.
     * Since we want to MINIMIZE error, we can model -error, or use Lower Confidence Bound.
     * Let's use Lower Confidence Bound (LCB) for minimization: Mean - beta * StdDev
     * We want to find x that minimizes LCB.
     */
    private double[] optimizeAcquisitionFunction() {
        // Simple random sampling (Monte Carlo) optimization of the acquisition function
        // Generate N candidate points, evaluate acquisition function, pick best.
        int numCandidates = 1000;
        double bestAcqValue = Double.MAX_VALUE;
        double[] bestCandidate = generateRandomParams(); // Default to random

        // Pre-compute K for the GP
        // K is matrix of kernel values between all observed points
        int n = X_observed.size();
        RealMatrix K = new Array2DRowRealMatrix(n, n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double k = kernel(X_observed.get(i), X_observed.get(j));
                if (i == j) k += noiseVariance;
                K.setEntry(i, j, k);
            }
        }

        try {
            // Cholesky is faster and more stable for GPs than LU
            CholeskyDecomposition cd = new CholeskyDecomposition(K);

            // Pre-compute alpha = K_inv * y
            RealVector y_vec = new ArrayRealVector(n);
            for(int i=0; i<n; i++) y_vec.setEntry(i, y_observed.get(i));
            RealVector alpha_vec = cd.getSolver().solve(y_vec);

            for (int i = 0; i < numCandidates; i++) {
                double[] candidate = generateRandomParams();

                // Predict mean and variance for candidate
                // k_star = vector of kernel(candidate, observed_i)
                RealVector k_star = new ArrayRealVector(n);
                for(int j=0; j<n; j++) {
                    k_star.setEntry(j, kernel(candidate, X_observed.get(j)));
                }

                double mean = k_star.dotProduct(alpha_vec);

                // Use the Cholesky solver to find variance efficiently
                // var = k(x,x) - k_star^T * K_inv * k_star
                // Let v = K_inv * k_star, which is solution to K * v = k_star
                RealVector v = cd.getSolver().solve(k_star);

                double k_xx = kernel(candidate, candidate) + noiseVariance;
                double var = k_xx - k_star.dotProduct(v);
                double std = Math.sqrt(Math.max(0, var));

                // LCB = Mean - beta * StdDev
                double lcb = mean - beta * std;

                if (lcb < bestAcqValue) {
                    bestAcqValue = lcb;
                    bestCandidate = candidate;
                }
            }
        } catch (Exception e) {
            // If matrix is not positive definite or singular, fallback to random
            return generateRandomParams();
        }

        return bestCandidate;
    }

    // Squared Exponential Kernel (RBF)
    private double kernel(double[] x1, double[] x2) {
        double distSq = 0;
        for (int i = 0; i < x1.length; i++) {
            double d = x1[i] - x2[i];
            distSq += d * d;
        }
        return signalVariance * Math.exp(-distSq / (2.0 * lengthScale * lengthScale));
    }

    public double[] getBestParams() {
        return bestParams;
    }

    public double getBestError() {
        return bestError;
    }

    public int getIteration() {
        return iteration;
    }

    public int getSampleCount() {
        return X_observed.size();
    }

    private double[] initialGuessNorm = null;

    public void setInitialGuess(double[] params) {
        this.initialGuessNorm = normalize(params);
    }

    private double[] normalize(double[] params) {
        double[] norm = new double[nParams];
        for(int i=0; i<nParams; i++) {
            norm[i] = (params[i] - bounds[i][0]) / (bounds[i][1] - bounds[i][0]);
        }
        return norm;
    }

    private double[] denormalize(double[] norm) {
        double[] params = new double[nParams];
        for(int i=0; i<nParams; i++) {
            params[i] = bounds[i][0] + norm[i] * (bounds[i][1] - bounds[i][0]);
        }
        return params;
    }
}


