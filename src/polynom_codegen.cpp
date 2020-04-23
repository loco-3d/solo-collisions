/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */
#include <iosfwd>
#include <cppad/cg.hpp>

using namespace CppAD;
using namespace CppAD::cg;

int main(void) {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    /* TEST : A POLYNOM */ 
    // Polynom coefficients : independent variable vector

    double p_coeff[5] = {2,4,2,1,1}; 

    CppAD::vector<ADCG> x(1);
    x[0] = 4.;
    Independent(x);

    CppAD::vector<ADCG> y(1);
    y[0] = 1;

    ADCG a = p_coeff[0] + p_coeff[1]*x[0] + p_coeff[2]*x[0]*x[0] ;
    y[0] = a;

    ADFun<CGD> fun(x, y); // the model tape   

    /***************************************************************************
     *                        Generate the C source code
     **************************************************************************/

    /**
     * start the special steps for source code generation for a Jacobian
     */
    CodeHandler<double> handler;

    CppAD::vector<CGD> indVars(1); // size of x (Independent variable)
    handler.makeVariables(indVars);

    CppAD::vector<CGD> gen_fun = fun.Forward(0, indVars);
    CppAD::vector<CGD> gen_jac = fun.SparseJacobian(indVars);

    LanguageC<double> langC("double");
    LangCDefaultVariableNameGenerator<double> nameGen;

    std::ostringstream code;
    //handler.generateCode(code, langC, jac, nameGen);
    handler.generateCode(code, langC, gen_fun, nameGen);

    std::ostringstream code_jac;
    handler.generateCode(code_jac, langC, gen_jac, nameGen);
    
    std::cout << "// Generated P(x) :\n";
    std::cout << code.str();
    std::cout << "// Generated P'(x) :\n";
    std::cout << code_jac.str();
}