#include <cstdlib>
#include <iostream>

int main()
{
  static_assert(__cplusplus >= 201703L, "Phase 0 requires C++17 or newer");
  std::cout << "repository_cpp_smoke: PASS\n";
  return EXIT_SUCCESS;
}
