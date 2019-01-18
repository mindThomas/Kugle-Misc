/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main( ){
 
    TIME autotime;
    DifferentialState q2;
    DifferentialState q3;
    DifferentialState x;
    DifferentialState y;
    DifferentialState dx;
    DifferentialState dy;
    DifferentialState s;
    DifferentialState ds;
    Control omega_ref_x;
    Control omega_ref_y;
    Control s_acceleration;
    OnlineData desiredVelocity; 
    OnlineData maxAngle; 
    OnlineData maxOmegaRef; 
    OnlineData minVelocity; 
    OnlineData maxVelocity; 
    OnlineData trajectoryLength; 
    OnlineData trajectoryStart; 
    OnlineData cx7; 
    OnlineData cx6; 
    OnlineData cx5; 
    OnlineData cx4; 
    OnlineData cx3; 
    OnlineData cx2; 
    OnlineData cx1; 
    OnlineData cx0; 
    OnlineData cy7; 
    OnlineData cy6; 
    OnlineData cy5; 
    OnlineData cy4; 
    OnlineData cy3; 
    OnlineData cy2; 
    OnlineData cy1; 
    OnlineData cy0; 
    IntermediateState intS1 = (s+trajectoryStart);
    IntermediateState intS2 = (cx0+cx1*intS1+cx2*pow(intS1,2.00000000000000000000e+00)+cx3*pow(intS1,3.00000000000000000000e+00)+cx4*pow(intS1,4.00000000000000000000e+00)+cx5*pow(intS1,5.00000000000000000000e+00)+cx6*pow(intS1,6.00000000000000000000e+00)+cx7*pow(intS1,7.00000000000000000000e+00));
    IntermediateState intS3 = (cy0+cy1*intS1+cy2*pow(intS1,2.00000000000000000000e+00)+cy3*pow(intS1,3.00000000000000000000e+00)+cy4*pow(intS1,4.00000000000000000000e+00)+cy5*pow(intS1,5.00000000000000000000e+00)+cy6*pow(intS1,6.00000000000000000000e+00)+cy7*pow(intS1,7.00000000000000000000e+00));
    BMatrix acadodata_M1;
    acadodata_M1.read( __SOURCE_FOLDER__"/kugle_mpc_export_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( __SOURCE_FOLDER__"/kugle_mpc_export_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << (-intS2+x);
    acadodata_f2 << (-intS3+y);
    acadodata_f2 << (-desiredVelocity+ds);
    acadodata_f2 << omega_ref_x;
    acadodata_f2 << omega_ref_y;
    Function acadodata_f3;
    acadodata_f3 << (-intS2+x);
    acadodata_f3 << (-intS3+y);
    acadodata_f3 << (-desiredVelocity+ds);
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(q2) == 5.00000000000000000000e-01*omega_ref_x;
    acadodata_f1 << dot(q3) == 5.00000000000000000000e-01*omega_ref_y;
    acadodata_f1 << dot(x) == dx;
    acadodata_f1 << dot(y) == dy;
    acadodata_f1 << dot(dx) == 1.37100000000000008527e+01*q3;
    acadodata_f1 << dot(dy) == (-1.37100000000000008527e+01)*q2;
    acadodata_f1 << dot(s) == ds;
    acadodata_f1 << dot(ds) == s_acceleration;

    OCP ocp1(0, 2.5, 50);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo((q2-sin(5.00000000000000000000e-01*maxAngle)) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-q2-sin(5.00000000000000000000e-01*maxAngle)) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((q3-sin(5.00000000000000000000e-01*maxAngle)) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-q3-sin(5.00000000000000000000e-01*maxAngle)) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef+omega_ref_x) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef-omega_ref_x) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef+omega_ref_y) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef-omega_ref_y) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((ds-minVelocity) >= 0.00000000000000000000e+00);
    ocp1.subjectTo((ds-maxVelocity) <= 0.00000000000000000000e+00);
    ocp1.setModel( acadodata_f1 );


    ocp1.setNU( 3 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 23 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 100 );
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    options_flag = ExportModule1.set( FIX_INITIAL_STATE, YES );
    uint export_flag;
    export_flag = ExportModule1.exportCode( "kugle_mpc_export" );

    if (!export_flag) {
	std::cout << "Exported MPC code successfully" << std::endl;
        std::cout << "Export code is located at: "__SOURCE_FOLDER__"../../build/kugle_mpc_export/" << std::endl;
    }

    return 0; 
 
} 

