#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/spatial/explog.hpp>

template<typename Scalar>
void calc(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& in,
          Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& out) {
  Eigen::Matrix<Scalar, 4, 4> M = Eigen::Map<const Eigen::Matrix<Scalar, 4, 4> >(in.data());
  pinocchio::SE3Tpl<Scalar> oMf(M);
  out = pinocchio::log6(oMf).toVector();
}

void new_calc(){
  
}

int main(void) {
  typedef double Scalar;

  typedef CppAD::cg::CG<Scalar> CGScalar;
  typedef CppAD::AD<CGScalar> ADScalar;
  typedef CppAD::ADFun<CGScalar> ADFun;

  //*********Setting up code gen variables and functions*********
  Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
  Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
  ad_X.resize(16);
  ad_Y.resize(6);
  ADFun ad_fun;
  CppAD::Independent(ad_X);
  calc(ad_X, ad_Y);
  ad_fun.Dependent(ad_X, ad_Y);
  ad_fun.optimize("no_compare_op");


  /******************Preparing Library********************/
  std::string function_name = "calc_cg";
  // Initialize Library
  CppAD::cg::ModelCSourceGen<Scalar> cgen(ad_fun, "calc_cg");
  cgen.setCreateForwardZero(true);
  cgen.setCreateJacobian(false);
  // cgen.setCreateForwardZero(true);
  CppAD::cg::ModelLibraryCSourceGen<Scalar> libcgen(cgen);
  CppAD::cg::DynamicModelLibraryProcessor<Scalar> dynamicLibManager(libcgen, "am_lib");
  // Compile Library
  CppAD::cg::GccCompiler<Scalar> compiler;
  std::vector<std::string> compile_options = compiler.getCompileFlags();
  compile_options[0] = "-Ofast";
  compiler.setCompileFlags(compile_options);
  dynamicLibManager.createDynamicLibrary(compiler, false);
  std::unique_ptr<CppAD::cg::DynamicLib<Scalar> > dynamicLib;
  // load Library
  const auto it = dynamicLibManager.getOptions().find("dlOpenMode");
  if (it == dynamicLibManager.getOptions().end()) {
    dynamicLib.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(dynamicLibManager.getLibraryName() +
                                                            CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
  } else {
    int dlOpenMode = std::stoi(it->second);
    dynamicLib.reset(new CppAD::cg::LinuxDynamicLib<Scalar>(
        dynamicLibManager.getLibraryName() + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION, dlOpenMode));
  }
  std::unique_ptr<CppAD::cg::GenericModel<Scalar> > generatedFun_ptr = dynamicLib->model(function_name.c_str());
  // Evaluate Function


  /******************************Computation*************************/

  Eigen::Matrix<Scalar, 4, 4> oMf;
  oMf << -0.928313, -0.119009, -0.352238, -0.0851641,
    -0.157173, -0.732959,  0.661867, 0.0303114,
    -0.336945,  0.669782, 0.66171, 0.190443,
    0.,0.,0.,1.;
  pinocchio::SE3Tpl<Scalar> M(oMf);//pinocchio::SE3Tpl<Scalar>::Random());  
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> v_in, v_out_cg, v_out;
  v_in.resize(16);
  v_out.resize(6);
  v_out_cg.resize(6);
  
  //M = pinocchio::SE3Tpl<Scalar>::Random();
  v_in = Eigen::Map<const Eigen::Matrix<Scalar, 16, 1> >(M.toHomogeneousMatrix().data(),
                                                         M.toHomogeneousMatrix().size());
    
  generatedFun_ptr->ForwardZero(v_in, v_out_cg);
  calc(v_in, v_out);
  std::cout << "Code Generated calc" << v_out_cg.transpose() << std::endl;
  std::cerr << "Original       calc" << v_out.transpose() << std::endl;

  std::cerr<<M<<std::endl;
  std::cerr<<Eigen::Map<const Eigen::Matrix<Scalar, 4, 4> >(v_in.data())<<std::endl;
}
