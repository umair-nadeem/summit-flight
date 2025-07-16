#pragma once

#include <variant>

#include "CortexFaultRecord.h"
#include "FreertosErrorRecords.h"
#include "GeneralErrorRecord.h"
#include "HalErrorRecord.h"

namespace error
{

using MasterErrorRecord = std::variant<std::monostate,
                                       CortexFaultRecord,
                                       FreertosStackOverflowRecord,
                                       FreertosAssertFailureRecord,
                                       HalErrorRecord,
                                       HalAssertFailureRecord,
                                       GeneralAssertRecord>;

void add_error_record(const MasterErrorRecord& error_record);
bool is_error_record_found();

}   // namespace error
