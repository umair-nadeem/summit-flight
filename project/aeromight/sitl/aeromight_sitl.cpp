#include "MainLoopData.hpp"
#include "aeromight_boundaries/AeromightData.hpp"
#include "error/error_handler.hpp"

namespace aeromight_boundaries
{
AeromightData aeromight_data{};
}   // namespace aeromight_boundaries

namespace sitl
{

MainLoopData main_loop_data{};

}   // namespace sitl

int main()
{
   sitl::run_main_loop(&(sitl::main_loop_data));
   return -1;
}
