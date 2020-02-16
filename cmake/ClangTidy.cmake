find_program(
    CLANG_TIDY_EXE
    NAMES clang-tidy clang-tidy-9 clang-tidy-8 clang-tidy-7 clang-tidy-6.0
    DOC "Path to clang-tidy executable"
)

if(CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY
        ${CLANG_TIDY_EXE}
        --checks=*,-abseil*,-android*,-fuchsia*,-modernize-use-trailing-return-type*,-zircon*,-objc*,-mpi*,-openmp*,-google-readability*,-readability*,-hicpp-braces*
    )
else()
    message(WARNING "No executable for clang-tidy found. Building without clang-tidy.")
endif()
