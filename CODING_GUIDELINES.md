
# Coding Guidelines

These coding standards are adapted from safety-critical development principles. The goal is to ensure predictable, maintainable, and testable embedded software.

## General Rules

1. **No dynamic memory allocation**
   - Avoid use of `new`, `delete`, `malloc`, `free`
   - Use statically allocated buffers and objects

2. **No raw pointers**
   - Prefer references or smart pointers with clear ownership
   - Use `std::optional`, `std::span`, `std::array` where applicable

3. **No class inheritance or virtual functions**
   - Use composition over inheritance
   - No runtime polymorphism (no vtables)

4. **No use of STL containers that allocate memory**
   - Avoid `std::vector`, `std::map`, etc.
   - Prefer `std::array`, `std::tuple`, `std::bitset`

5. **All objects should be created at compile-time where possible**
   - Initialization should not depend on runtime allocations

## Testing Rules

- Unit tests may use dynamic memory if they run on host machine only
- GoogleTest is allowed for off-target test modules
