PROJECT(rlplan)

SET(Boost_ADDITIONAL_VERSIONS "1.37" "1.37.0" "1.38" "1.38.0" "1.39" "1.39.0")
FIND_PACKAGE(Boost REQUIRED)

FIND_PACKAGE(BoostNumericBindings REQUIRED)
FIND_PACKAGE(BoostNumericBindingsIpp REQUIRED)
FIND_PACKAGE(Cgal REQUIRED)
FIND_PACKAGE(Coin REQUIRED)
FIND_PACKAGE(Iconv REQUIRED)
FIND_PACKAGE(Ipp REQUIRED)
FIND_PACKAGE(Libxml2 REQUIRED)
#FIND_PACKAGE(Ode REQUIRED)
#FIND_PACKAGE(Pqp REQUIRED)
FIND_PACKAGE(RlKin REQUIRED)
FIND_PACKAGE(RlMath REQUIRED)
FIND_PACKAGE(RlSg REQUIRED)
FIND_PACKAGE(RlUtil REQUIRED)
FIND_PACKAGE(RlXml REQUIRED)
FIND_PACKAGE(Solid REQUIRED)
#FIND_PACKAGE(Swift++ REQUIRED)
FIND_PACKAGE(Zlib REQUIRED)

ADD_DEFINITIONS(
	-D_SCL_SECURE_NO_WARNINGS
	-DCOIN_NOT_DLL
	-DNOMINMAX
	${RL_UTIL_DEFINITIONS}
)

INCLUDE_DIRECTORIES(
	BEFORE
	${Boost_INCLUDE_DIR}
	${BOOSTNUMERICBINDINGS_INCLUDE_DIR}
	${BOOSTNUMERICBINDINGSIPP_INCLUDE_DIR}
	${CGAL_INCLUDE_DIRS}
	${COIN_INCLUDE_DIR}
	${ICONV_INCLUDE_DIR}
	${IPP_INCLUDE_DIR}
	${LIBXML2_INCLUDE_DIR}
#	${ODE_INCLUDE_DIR}
#	${PQP_INCLUDE_DIR}
	${RL_KIN_INCLUDE_DIR}
	${RL_MATH_INCLUDE_DIR}
	${RL_SG_INCLUDE_DIR}
	${RL_UTIL_INCLUDE_DIR}
	${RL_XML_INCLUDE_DIR}
	${SOLID_INCLUDE_DIR}
#	${SWIFTPP_INCLUDE_DIR}
	${ZLIB_INCLUDE_DIR}
)

SET(
	HDRS
	AddRrtConCon.h
	AdvancedOptimizer.h
	BridgeSampler.h
	DistanceModel.h
	GaussianSampler.h
	MatrixPtr.h
	Model.h
	Optimizer.h
	Planner.h
	Prm.h
	RecursiveVerifier.h
	Rrt.h
	RrtCon.h
	RrtMod.h
	RrtConCon.h
	RrtDual.h
	RrtExtCon.h
	RrtExtExt.h
	RrtGoalBias.h
	Sampler.h
	SimpleModel.h
	SimpleOptimizer.h
	TransformPtr.h
	UniformSampler.h
	VectorList.h
	VectorPtr.h
	Verifier.h
	Viewer.h
	WorkspaceSphere.h
	WorkspaceSphereExplorer.h
	WorkspaceSphereList.h
)

SET(
	SRCS
	AddRrtConCon.cpp
	AdvancedOptimizer.cpp
	BridgeSampler.cpp
	DistanceModel.cpp
	GaussianSampler.cpp
	Model.cpp
	Optimizer.cpp
	Planner.cpp
	Prm.cpp
	RecursiveVerifier.cpp
	Rrt.cpp
	RrtCon.cpp
	RrtMod.cpp
	RrtConCon.cpp
	RrtDual.cpp
	RrtExtCon.cpp
	RrtExtExt.cpp
	RrtGoalBias.cpp
	Sampler.cpp
	SimpleModel.cpp
	SimpleOptimizer.cpp
	UniformSampler.cpp
	Verifier.cpp
	Viewer.cpp
	WorkspaceSphere.cpp
	WorkspaceSphereExplorer.cpp
)

IF (WIN32)
	ADD_LIBRARY(
		rlplan
		STATIC
		${HDRS}
		${SRCS}
	)
ELSE (WIN32)
	ADD_LIBRARY(
		rlplan
		SHARED
		${HDRS}
		${SRCS}
	)
ENDIF (WIN32)

TARGET_LINK_LIBRARIES(
	rlplan
	${CGAL_LIBRARY}
	${COIN_LIBRARY}
	${IPP_LIBRARIES}
	${LIBXML2_LIBRARY}
#	${ODE_LIBRARY}
#	${PQP_LIBRARY}
	${RL_KIN_LIBRARY}
	${RL_SG_LIBRARY}
	${SOLID_LIBRARY}
#	${SWIFTPP_LIBRARY}
)

SET_TARGET_PROPERTIES(
	rlplan
	PROPERTIES
	VERSION 0.5
	DEBUG_POSTFIX d
)

INSTALL(FILES ${HDRS} DESTINATION include/rl/plan)

INSTALL(
	TARGETS
	rlplan
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
