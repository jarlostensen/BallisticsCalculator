#include "BulletData.h"
#include <sstream>

namespace Ballistics
{
    namespace
    {
        // trim leading whitespace *and* opening and closing brackets
        std::string Trim(const std::string& str) {
            size_t start = str.find_first_not_of(" \t\n\r{");
            if (start == std::string::npos) return "";
    
            size_t end = str.find_last_not_of(" \t\n\r}");
            return str.substr(start, end - start + 1);
        }
    }
    
    bool BulletData::ParseFromJsonString(const std::string& InJsonString)
    {
        std::string JsonString(Trim(InJsonString));
        if ( JsonString.empty() )
        {
            return false;
        }
        
        std::stringstream JsonStringStream(JsonString);
        std::string Line;
        while (std::getline(JsonStringStream, Line))
        {
            std::string Key, Value;
            size_t DelimeterPos = Line.find(':');
            if (DelimeterPos != std::string::npos)
            {
                Key = Trim(Line.substr(0, DelimeterPos));
                const size_t NextDelimeterPos = Line.find(',', DelimeterPos + 1);
                Value = NextDelimeterPos != std::string::npos ?
                Trim(Line.substr(DelimeterPos + 1, NextDelimeterPos - DelimeterPos - 1))
                :
                Trim(Line.substr(DelimeterPos + 1, Line.size() - DelimeterPos - 1));

                // strip leading and trailing "'s
                Key = Key.substr(1, Key.size() - 2);
                Value = Value.substr(1, Value.size() - 2);
            }
            else
            {
                continue;
            }

            if ( Value.empty())
            {
                continue;
            }

            if (Key == "product_name")
            {
                Name = Value;
            }
            else if (Key == "description")
            {
                Description = Value;
            }
            else if (Key == "company")
            {
                Company = Value;
            }
            else if (Key == "diameter_in")
            {
                CallibreMm = std::stof(Value) * 24.4f;
            }
            else if (Key == "weight_gr")
            {
                MassGr = std::stof(Value);
            }
            else if (Key == "bc_g1")
            {
                G1BC = std::stof(Value);
            }
            else if (Key == "bc_g7")
            {
                G7BC = std::stof(Value);
            }
        }
        return true;
    }
}
