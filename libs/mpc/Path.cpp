/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "Path.h"

#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include <cmath>
#include <algorithm>

#include <Eigen/SVD>

/* For visualization/plotting only */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PATH_DEBUG  0


namespace MPC
{

    Polynomial::Polynomial()
    {

    }

    // Copy constructor
    Polynomial::Polynomial(const Polynomial& poly) : coeffs_(poly.coeffs_)
    {

    }

    Polynomial::~Polynomial()
    {

    }

    Polynomial::Polynomial(unsigned int order) : coeffs_(order+1, 0.0)
    {

    }

    /* Coefficients are stored such that c[0] is the coefficient for the lowest order element, such that:
     * y = c[n]*x^n + c[n-1]*x^(n-1) + ... c[2]*x^2 + c[1]*x + c[0]
     */
    Polynomial::Polynomial(std::vector<double> coeffs)
    {
        coeffs_ = coeffs;
    }

    Polynomial::Polynomial(const double * coeffs, int num_coeffs)
    {
        coeffs_.insert(coeffs_.end(), coeffs, coeffs + num_coeffs);
    }

    // Assignment operator
    Polynomial& Polynomial::operator=(const Polynomial& other)
    {
        coeffs_ = other.coeffs_;
        return *this;
    }

    Polynomial Polynomial::operator+(const Polynomial& other) const
    {
        Polynomial out;

        if (order() >= other.order()) {
            out.coeffs_ = coeffs_;
            for (size_t i = 0; i < other.coeffs_.size(); i++)
                out.coeffs_.at(i) += other.coeffs_.at(i);
        }
        else {
            out.coeffs_ = other.coeffs_;
            for (size_t i = 0; i < coeffs_.size(); i++)
                out.coeffs_.at(i) += coeffs_.at(i);
        }

        return out;
    }

    Polynomial Polynomial::operator-(const Polynomial& other) const
    {
        Polynomial out;

        if (order() >= other.order()) {
            out.coeffs_ = coeffs_;
            for (size_t i = 0; i < other.coeffs_.size(); i++)
                out.coeffs_.at(i) -= other.coeffs_.at(i);
        }
        else {
            out.coeffs_ = other.coeffs_;
            for (size_t i = 0; i < coeffs_.size(); i++)
                out.coeffs_.at(i) -= coeffs_.at(i);
        }

        return out;
    }

    Polynomial Polynomial::operator+(const double& offset) const
    {
        Polynomial out;
        if (coeffs_.size() == 0) return out;

        out.coeffs_ = coeffs_;
        out.coeffs_[0] += offset;
        return out;
    }

    Polynomial Polynomial::operator-(const double& offset) const
    {
        Polynomial out;
        if (coeffs_.size() == 0) return out;

        out.coeffs_ = coeffs_;
        out.coeffs_[0] -= offset;
        return out;
    }

    void Polynomial::print()
    {
        std::cout << "Polynomial:" << std::endl;
        std::cout << "   y = ";
        if (order() < 0) {
            std::cout << "0" << std::endl << std::endl;
            return;
        }

        for (int i = order(); i >= 1; i--) {
            if (i > 1)
                std::cout << coeffs_.at(i) << "*t^" << i << " + ";
            else
                std::cout << coeffs_.at(i) << "*t + ";
        }
        std::cout << coeffs_.at(0) << std::endl;
        std::cout << std::endl;
    }

    /* Coefficients are stored such that c[0] is the coefficient for the lowest order element, such that:
     * out = c[n]*t^n + c[n-1]*t^(n-1) + ... c[2]*t^2 + c[1]*t + c[0]
     */
    double Polynomial::evaluate(double t)
    {
        // Polynomial evaluation
        double pow_t = 1;
        double out = 0;
        for (unsigned int i = 0; i < coeffs_.size(); i++) {
            out += coeffs_[i] * pow_t;
            pow_t *= t;
        }

        return out;
    }

    double Polynomial::operator()(double t)
    {
        return evaluate(t);
    }

    std::vector<double> Polynomial::evaluate(std::vector<double> tVec)
    {
        std::vector<double> evaluatedPoints;

        // Polynomial evaluation
        for (auto& t : tVec) {
            double pow_t = 1;
            double out = 0;
            for (unsigned int i = 0; i < coeffs_.size(); i++) {
                pow_t *= t;
                out += coeffs_[i] * pow_t;
            }

            evaluatedPoints.push_back(out);
        }

        return evaluatedPoints;
    }

    std::vector<double> Polynomial::operator()(std::vector<double> tVec)
    {
        return evaluate(tVec);
    }

    int Polynomial::order() const
    {
        return (int)coeffs_.size() - 1;
    }

    Polynomial Polynomial::squared() const
    {
        // Given polynomial coefficients for:
        // f(x) = c_n*x^n + c_n-1*x^(n-1) + ... c_1*x + c_0
        // This function computes the coefficients of the polynomial f^2(x)
        // Coefficients are ordered such that coeffs[0] = c_0  (lowest order)

        // f^2(x) = sum(sum(c_i * c_j * x^(i+j))
        // The resulting coefficients, coeff_k = c_i*c_j | i+j=k
        int n = order(); // input polynomial order
        int m = 2*n; // squared (output) polynomial order

        if (m < 0) {
            Polynomial empty;
            return empty;
        }

        Polynomial squared(m);
        for (int k = 0; k <= m; k++) {
            for (int i = 0; i <= n; i++) {
                for (int j = 0; j <= n; j++) {
                    if (i + j == k) {
                        squared.coeffs_.at(k) = squared.coeffs_.at(k) + coeffs_.at(i) * coeffs_.at(j);
                    }
                }
            }
        }
        return squared;
    }

    Polynomial Polynomial::derivative() const
    {
        // Given polynomial coefficients for:
        // f(x) = c_n*x^n + c_n-1*x^(n-1) + ... c_1*x + c_0
        // This function computes the coefficients of the polynomial f'(x)
        // f'(x) = df/dx = n*c_n*x^(n-1) + (n-1)*c_n-1*x^(n-2) + ... +
        // f'(x) = df/dx = n*c_n*x^(n-1) + (n-1)*c_n-1*x^(n-2) + ... + 2*c_2*x + 1*c_1
        // Coefficients are ordered such that coeff[0] = c_0  (lowest order)
        int n = order();

        if (n < 0) {
            Polynomial empty;
            return empty;
        }

        Polynomial derivative(n-1);
        for (int i = 0; i <= (n-1); i++) {
            derivative.coeffs_.at(i) = (i+1) * coeffs_.at(i+1);
        }
        return derivative;
    }

    double Polynomial::getCoefficient(unsigned int index)
    {
        if (int(index) > order())
            return 0;
        else
            return coeffs_.at(index);
    }

    double Polynomial::findMinimum(double s_init, double s_lower, double s_upper, double stoppingCriteria, unsigned int maxIterations)
    {
        // Find the minimum by Newton minimization
        if (order() < 0) return 0;

        Polynomial dPoly = derivative();
        Polynomial ddPoly = dPoly.derivative();

        // Newton's method - https://en.wikipedia.org/wiki/Newton%27s_method_in_optimization
        bool converged = false;
        unsigned int iterations = 0;
        double s = s_init;
        double delta_s;

        while (!converged && iterations < maxIterations) {
            double FirstDerivative = dPoly.evaluate(s); // first derivative, for higher dimensions it would be the Gradient
            double SecondDerivative = ddPoly.evaluate(s); // second derivative, for higher dimensions it would be the Hessian

            delta_s = -FirstDerivative / SecondDerivative;
            s = s + delta_s;

            if (s < s_lower)
                s = s_lower;

            if (s > s_upper)
                s = s_upper;

            if (std::abs(delta_s) < stoppingCriteria)
                converged = true;

            iterations = iterations + 1;
        }

        return s;
    }

    void Polynomial::FitPoints(unsigned int order, std::vector<double>& tVec, std::vector<double>& values, bool EnforceBeginEndConstraint, bool EnforceBeginEndAngleConstraint)
    {
        if (order < 1) {
            std::cout << "Polynomial order needs to be at least 1" << std::endl; throw;
        }
        else if (order < 3 && EnforceBeginEndConstraint && EnforceBeginEndAngleConstraint) {
            std::cout << "Polynomial order too low to enforce both type of constraints" << std::endl; throw;
        }
        if (tVec.size() != values.size()) {
            std::cout << "Number of evaluation points (tVec) and corresponding values needs to be consistent" << std::endl; throw;
        }

        unsigned int n = tVec.size(); // should be the same length as values

        Eigen::VectorXd t = Eigen::VectorXd::Map(tVec.data(), tVec.size());
        Eigen::VectorXd tPow = Eigen::VectorXd::Ones(n, 1);

        Eigen::MatrixXd A(n, order+1);
        // Fill the matrix using the evaluation points, t
        for (unsigned int i = 0; i <= order; i++) {
            A.block(0, i, n, 1) = tPow;
            tPow = tPow.cwiseProduct(t); // tPow = tPow .* tVec
        }

        Eigen::VectorXd b = Eigen::VectorXd::Map(values.data(), values.size());

        // Create matrix and vector of equality constraints
        Eigen::MatrixXd Aeq;
        Eigen::VectorXd beq;
        if (EnforceBeginEndConstraint || EnforceBeginEndAngleConstraint) { // compute the constraint matrices
            Aeq.resize(4, order+1); // OBS. Note that resizing does not initialize values!
            beq.resize(4, 1);

            // First two rows enforces begin and end constraints
            Aeq(0,0) = 1;
            Aeq(1,0) = 1;
            // Last two rows enforces begin and end angle (derivative) constraints
            Aeq(2,0) = 0;
            Aeq(3,0) = 0;

            for (unsigned int j = 1; j <= order; j++) {
                Aeq(0, j) = Aeq(0, j-1) * t(0);
                Aeq(1, j) = Aeq(1, j-1) * t(n-1);
                Aeq(2, j) = Aeq(0, j-1) * j;
                Aeq(3, j) = Aeq(1, j-1) * j;
            }

            beq(0) = b(0); // first value
            beq(1) = b(n-1); // last value
            beq(2) = (b(1)-b(0)) / (t(1)-t(0)); // begin angle
            beq(3) = (b(n-1)-b(n-2)) / (t(n-1)-t(n-2)); // end angle
        }

#if PATH_DEBUG
        std::cout << "Aeq = " << std::endl << Aeq << std::endl;
        std::cout << "beq = " << std::endl << beq << std::endl;
#endif

        if (order < 3) { // can only enforce one type of the constraints
            // limit constraints due to reduced order such that we only require start and end point to be fulfilled
            if (EnforceBeginEndConstraint) {
                // Extract the first two rows
                Aeq = Aeq.block(0, 0, 2, order+1);
                beq = beq.block(0, 0, 2, 2);
            } else if (EnforceBeginEndAngleConstraint) {
                // Extract the last two rows
                Aeq = Aeq.block(2, 0, 2, order+1);
                beq = beq.block(2, 0, 2, 2);
            }
        }

        Eigen::VectorXd polyCoeffs;
        if (EnforceBeginEndConstraint || EnforceBeginEndAngleConstraint)
            polyCoeffs = ConstrainedLeastSquares(A, b, Aeq, beq, n*10000);
        else
            polyCoeffs = ConstrainedLeastSquares(A, b, Eigen::MatrixXd(), Eigen::VectorXd(), 1);

#if PATH_DEBUG
        std::cout << "polyCoeffs = " << std::endl << polyCoeffs << std::endl;
#endif

        coeffs_.clear();
        coeffs_.insert(coeffs_.begin(), polyCoeffs.data(), polyCoeffs.data() + polyCoeffs.rows());
    }

    Eigen::VectorXd Polynomial::ConstrainedLeastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq, double lambda)
    {
        Eigen::MatrixXd A_;
        Eigen::VectorXd b_;

        if (Aeq.rows() == beq.rows() && Aeq.cols() == A.cols()) {
            A_.resize(A.rows() + Aeq.rows(), A.cols()); // OBS. Note that resizing does not initialize values!
            b_.resize(b.rows() + beq.rows());
            A_ << A,
                  Aeq*lambda;
            b_ << b,
                  beq*lambda;
        } else {
            A_ = A;
            b_ = b;
        }

#if PATH_DEBUG
        std::cout << "A_ = " << std::endl << A_ << std::endl;
#endif

        // Pseudo-inverse through numerically unstable way
        //   A_invpseudo = inv(A_'*A_) * A_'
        // Pseudo-inverse through MATLAB
        //   A_invpseudo2 = pinv(A_)  % Moore-Penrose Pseudoinverse of matrix of A
        // Pseudo-inverse through SVD
        //   [U,S,V] = svd(A_);
        //   Sinv = [diag(1./diag(S)), zeros(size(S,2), size(S,1)-size(S,2))];
        //   A_invpseudo3 = V * Sinv * U';
        //   x = A_invpseudo3 * b_;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd( A_, Eigen::ComputeFullV | Eigen::ComputeFullU );

        // create a matrix of just zeros and fill top part with inverse singular values in a diagonal matrix
        Eigen::MatrixXd Sinv = Eigen::MatrixXd::Zero(svd.singularValues().rows(), A_.rows());
        Sinv.block(0, 0, svd.singularValues().rows(), svd.singularValues().rows()) = svd.singularValues().asDiagonal().inverse().toDenseMatrix();
#if PATH_DEBUG
        std::cout << "Sinv = " << std::endl << Sinv << std::endl;
#endif

        Eigen::MatrixXd A_PseudoInverse = svd.matrixV() * Sinv * svd.matrixU().transpose();
#if PATH_DEBUG
        std::cout << "A_PseudoInverse = " << std::endl << A_PseudoInverse << std::endl;
#endif

        // Now get the solution (least squares minimization) by using the pseudo-inverse
        Eigen::VectorXd x = A_PseudoInverse * b_;
#if PATH_DEBUG
        std::cout << "x = " << std::endl << x << std::endl;
#endif

        return x;
    }

    Path::Path() : computed_dsquared_(false), s_end_(0)
    {

    }

    Path::Path(Polynomial& poly_x, Polynomial& poly_y, double s_end) : poly_x_(poly_x), poly_y_(poly_y), s_end_(s_end), computed_dsquared_(false)
    {

    }

    Path::Path(Trajectory& trajectory, unsigned int approximationOrder, bool StopAtEnd, bool EnforceBeginEndConstraint, bool EnforceBeginEndAngleConstraint) : computed_dsquared_(false)
    {
        FitTrajectory(trajectory, approximationOrder, StopAtEnd, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);
    }

    Path::~Path()
    {

    }

    // Assignment operator
    Path& Path::operator=(const Path& other)
    {
        poly_x_ = other.poly_x_;
        poly_y_ = other.poly_y_;
        s_end_ = other.s_end_;
        computed_dsquared_ = other.computed_dsquared_;
        dsquared_ = other.dsquared_;
        return *this;
    }

    Eigen::Vector2d Path::get(double s)
    {
        return Eigen::Vector2d(poly_x_.evaluate(s), poly_y_.evaluate(s));
    }

    Eigen::Vector2d Path::operator()(double s)
    {
        return get(s);
    }

    std::vector<Eigen::Vector2d> Path::get(std::vector<double> sVec)
    {
        std::vector<Eigen::Vector2d> evaluatedPoints;

        // Polynomial evaluation
        for (auto& s : sVec) {
            evaluatedPoints.push_back(Eigen::Vector2d(poly_x_.evaluate(s), poly_y_.evaluate(s)));
        }

        return evaluatedPoints;
    }

    std::vector<Eigen::Vector2d> Path::operator()(std::vector<double> sVec)
    {
        return get(sVec);
    }

    int Path::order()
    {
        if (poly_x_.order() >= poly_y_.order())
            return poly_x_.order();
        else
            return poly_y_.order();
    }

    double Path::getXcoefficient(unsigned int index)
    {
        return poly_x_.getCoefficient(index);
    }

    double Path::getYcoefficient(unsigned int index)
    {
        return poly_y_.getCoefficient(index);
    }

    double Path::ArcCurveLength(double t)
    {
        // Arc curve length of a polynomial path whose position is defined as:
        //   p(t) = [x(t), y(t)]
        // Is given by the integral:
        // integral( sqrt( dxdt^2 + dydt^2 ) ) dt
        // This can however be approximated by using the velocity polynomial
        //   Q = sqrt( (df_x/dx)^2 + (df_dy)^2 );

        // First we compute the coefficients of the inner polynomial, f
        //   f = (df_x/dx)^2 + (df_dy)^2
        if (!computed_dsquared_) {
            Polynomial dx = poly_x_.derivative(); // taking the difference of a polynomial, moves the coefficients
            Polynomial dy = poly_y_.derivative();
            Polynomial dx_squared = dx.squared();
            Polynomial dy_squared = dy.squared();
            dsquared_ = dx_squared + dy_squared;
            computed_dsquared_ = true;
        }
        // Such that
        // Q = sqrt(EvaluatePolynomial(f_coeff, t))

        // Return the approximated arc curve length at point t - hence curve length from f(0) to f(t)
        // Based on http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.5.2912&rep=rep1&type=pdf
        // The approximation (see paper above) is given by
        // s(t) = t/2 * (5/9 * Q(1.774597*t/2) + 8/9 * Q(t/2) + 5/9 * Q(0.225403*t/2))
        double s = t/2.0 * (
                            5.0/9.0 * sqrt(dsquared_.evaluate(1.774597*t/2.0)) +
                            8.0/9.0 * sqrt(dsquared_.evaluate(t/2.0)) +
                            5.0/9.0 * sqrt(dsquared_.evaluate(0.225403*t/2.0))
                           );
        return s;
    }

    double Path::ApproximateArcCurveLength(double t, double discretizationStepSize)
    {
        double distance = 0;
        double t_current = 0;

        double x_prev = poly_x_.evaluate(t_current);
        double y_prev = poly_y_.evaluate(t_current);
        double x, y;
        double x_diff, y_diff;

        while (t_current < t) {
            t_current += discretizationStepSize;
            x = poly_x_.evaluate(t_current);
            y = poly_y_.evaluate(t_current);

            x_diff = x - x_prev;
            y_diff = y - y_prev;

            distance += sqrtf(x_diff*x_diff + y_diff*y_diff); // Euclidean distance

            x_prev = x;
            y_prev = y;
        }

        return distance;
    }

    void Path::plot(bool drawXup, double x_min, double y_min, double x_max, double y_max)
    {
        double aspect_ratio = (x_max - x_min) / (y_max - y_min);
        double xres, yres;
        if (aspect_ratio >= 1) {
            yres = 500;
            xres = yres * aspect_ratio;
        } else {
            xres = 500;
            yres = xres / aspect_ratio;
        }

        // Create black empty images
        cv::Mat image;
        if (drawXup)
            image = cv::Mat( xres, yres, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
        else
            image = cv::Mat( yres, xres, CV_8UC3, cv::Scalar( 255, 255, 255 ) );

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        if (s_end_ <= 0) return; // can not plot if length is unknown
        double s_spacing = s_end_ / 99.0;

        Eigen::Vector2d p_prev = get(0);
        Eigen::Vector2d p;

        for (unsigned int s_idx = 0; s_idx < 100; s_idx++) {
            double s = s_spacing * s_idx;
            p = get(s);

            float x = (p[0]-x_min) * scale_x;
            float y = (p[1]-y_min) * scale_y;

            if (x >= 0 && x < xres && y >= 0 && y < yres) {
                cv::Point point;
                if (drawXup) // draw with robot x-axis pointing up in plot
                    point = cv::Point(yres-y,xres-x);
                else
                    point = cv::Point(x,yres-y);

                // Draw a line
                //cv::line(image, cv::Point(p_prev[0], p_prev[1]), point, cv::Scalar( 0, 0, 0 ), 1, 8 );
                cv::drawMarker(image, point, cv::Scalar( 255, 0, 0 ), cv::MARKER_CROSS, 3, 1, 8 );

                p_prev = p;
            }
        }

        cv::imshow("Path", image);

        cv::waitKey( 5 );
    }

    void Path::plot(cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max)
    {
        double xres = image.cols;
        double yres = image.rows;

        cv::line(image, cv::Point(0, yres/2), cv::Point(xres-1, yres/2), cv::Scalar(128,128,128), 1, 8, 0);
        cv::line(image, cv::Point(xres/2, 0), cv::Point(xres/2, yres-1), cv::Scalar(128,128,128), 1, 8, 0);

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        if (s_end_ <= 0) return; // can not plot if length is unknown
        double s_spacing = s_end_ / 99.0;

        Eigen::Vector2d p_prev = get(0);
        Eigen::Vector2d p;

        for (unsigned int s_idx = 0; s_idx < 100; s_idx++) {
            double s = s_spacing * s_idx;
            p = get(s);

            float x = (p[0]-x_min) * scale_x;
            float y = (p[1]-y_min) * scale_y;

            if (x >= 0 && x < xres && y >= 0 && y < yres) {
                cv::Point point;
                if (drawXup) // draw with robot x-axis pointing up in plot
                    point = cv::Point(yres-y,xres-x);
                else
                    point = cv::Point(x,yres-y);

                // Draw a line
                //cv::line(image, cv::Point(p_prev[0], p_prev[1]), point, cv::Scalar( 0, 0, 0 ), 1, 8 );
                cv::drawMarker(image, point, color, cv::MARKER_CROSS, 3, 1, 8 );

                p_prev = p;
            }
        }
    }

    void Path::PlotPoint(double sValue, cv::Mat& image, cv::Scalar color, bool drawXup, double x_min, double y_min, double x_max, double y_max)
    {
        double xres = image.cols;
        double yres = image.rows;
        Eigen::Vector2d p = get(sValue);

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        float x = (p[0]-x_min) * scale_x;
        float y = (p[1]-y_min) * scale_y;

        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            cv::Point point;
            if (drawXup) // draw with robot x-axis pointing up in plot
                point = cv::Point(yres-y,xres-x);
            else
                point = cv::Point(x,yres-y);

            cv::drawMarker(image, point, color, cv::MARKER_STAR, 6, 2, 8);
        }
    }

    void Path::FitTrajectory(Trajectory& trajectory, unsigned int approximationOrder, bool StopAtEnd, bool EnforceBeginEndConstraint, bool EnforceBeginEndAngleConstraint)
    {
        // approximationOrder is for x(t) and y(t) fitting
        unsigned order_t2s = approximationOrder + 1; // order_t2s is for t(s) fitting
        unsigned order_f2s = approximationOrder + 1; // order_f2s is the order for the final fitted path: x(s) and y(s)

        std::vector<double> tVec = trajectory.GetDistanceList(); // get approximated distance vector for the individual points in the trajectory
        std::vector<double> xValues = trajectory.GetX();
        std::vector<double> yValues = trajectory.GetY();

        if (tVec.size() <= order_t2s) { // we do not have sufficient points for fitting - therefore just do a path that corresponds to holding a static position (end point)
            s_end_ = 99;
            poly_x_ = Polynomial({trajectory.back().point[0], 1000});
            poly_y_ = Polynomial({trajectory.back().point[1], 1000});
            return;
        }

        /* Fit window points to two polynomials, x(t) and y(t), using parameters t starting with t0=0 and spaced with the distance between each point (chordal parameterization) */
        Polynomial xt, yt;
        xt.FitPoints(approximationOrder, tVec, xValues, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);
        yt.FitPoints(approximationOrder, tVec, yValues, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);

        // Create temporary path holding the x(t) and y(t) approximation
        Path tmpPath(xt, yt);

        /* Compute numerical approximation of arc length at the points, t, using the fitted polynomial */
        // Given the now approximated x(t), y(t) path, we approximate the arc curve length for the different points
        // Hence to be able to create a mapping between s to t : t(s)
        std::vector<double> sVec;
        for (double& t : tVec) {
            sVec.push_back(tmpPath.ArcCurveLength(t));
        }

        double sTotal = sVec.back();

        /* Fit t(s) polynomial */
        Polynomial ts;
        ts.FitPoints(order_t2s, sVec, tVec, true, false);

        /* Create vector of evenly spaced distances, s, and get corresponding t values */
        // Make 100 linearly (evenly) seperated distance points
        double spacing = sTotal / 99;
        std::vector<double> sEven;
        std::vector<double> tEven;
        for (int i = 0; i < 100; i++) {
            double s = i * spacing;
            sEven.push_back(s);
            tEven.push_back(ts.evaluate(s));
        }

        /* Use vector of the corresponding t values (matching the evenly spaced s distances) to get corresponding x-y value pairs from the initial approximation */
        std::vector<double> xEven;
        std::vector<double> yEven;
        for (double& t : tEven) {
            xEven.push_back(xt.evaluate(t));
            yEven.push_back(yt.evaluate(t));
        }

        /* Hack to make the fitted trajectory keep the same position after reaching the distance value */
        /*if (StopAtEnd) {
            for (int i = 1; i < 100; i++) {
                double s = sTotal + i * spacing;
                sEven.push_back(s);
                xEven.push_back(xEven.back() + spacing/1000);
                yEven.push_back(yEven.back());
            }
        }*/

        /* Use the s to x-y pairs to create final approximation: x(s) and y(s) */
        // Fit two new polynomials on the new generated points, x_0,...,x_n  and y_0,...,y_n  using the evenly spaced distance parameters, s_0,...,s_n, as the parameter
        poly_x_.FitPoints(order_f2s, sEven, xEven, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);
        poly_y_.FitPoints(order_f2s, sEven, yEven, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);
        s_end_ = sEven.back(); // sTotal
    }

    void Path::print()
    {
        std::cout << "x(s) ";
        poly_x_.print();
        std::cout << "y(s) ";
        poly_y_.print();
    }

    double Path::length()
    {
        return s_end_;
    }

    double Path::FindClosestPoint(const Eigen::Vector2d& position)
    {
        // Find point on path being closest to the input position and return the corresponding path parameter (s-value) at this point
        // First we create a centered path around the input position, by taking the two polynomial and subtracting the position value
        Polynomial xs_centered = poly_x_ - position[0];
        Polynomial ys_centered = poly_y_ - position[1];
        // Given this centered polynomial, the distance to any point along the original path is defined by the function:
        //   dist(s) = sqrt( f_x(s)^2 + f_y(s)^2 )
        // Since this is the function we want to minimize we create this function as a distance polynomial such that we can take the derivative and find the minimum point
        Polynomial xs_squared = xs_centered.squared();
        Polynomial ys_squared = ys_centered.squared();
        // And since we want to find the closest distance, we can also just
        // minimize the squared distance: dist(s)^2
        //   dist(s)^2 = f_x(s)^2 + f_y(s)^2
        Polynomial dist_squared = xs_squared + ys_squared;

        // Finally we find the minimum distance, s, through a Newton-based minimization
        return dist_squared.findMinimum(0, 0, s_end_, 0.001, 100);
    }

}
