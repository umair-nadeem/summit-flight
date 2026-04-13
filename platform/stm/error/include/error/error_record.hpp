#pragma once

#include <variant>

#include "CortexFaultRecord.hpp"
#include "FreertosErrorRecords.hpp"
#include "GeneralErrorRecord.hpp"
#include "HalErrorRecord.hpp"

namespace error
{

using MasterErrorRecord = std::variant<std::monostate,
                                       CortexFaultRecord,
                                       FreertosStackOverflowRecord,
                                       FreertosAssertFailureRecord,
                                       HalErrorRecord,
                                       HalAssertFailureRecord,
                                       GeneralFailureRecord>;

void add_error_record(const MasterErrorRecord& error_record);
bool has_no_error();

}   // namespace error
