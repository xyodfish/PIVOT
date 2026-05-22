/**
 * @file planner_exception.h
 * @brief Velocity Planning Module - Exception Handling
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_PLANNER_EXCEPTION_H
#define VP_PLANNER_EXCEPTION_H

#include <exception>
#include <string>
#include <unordered_map>

namespace vp {

/**
 * @brief Exception class for velocity planning errors
 */
class PlannerException : public std::exception {
   private:
    int error_code_;

   public:
    /**
     * @brief Construct a new Planner Exception
     * @param code Error code
     */
    explicit PlannerException(int code) : error_code_(code) {}

    ~PlannerException() override = default;

    /**
     * @brief Get error message
     * @return Error description string
     */
    const char* what() const noexcept override {
        return getErrorMessage(error_code_).c_str();
    }

    /**
     * @brief Get error code
     * @return Error code
     */
    int errorCode() const { return error_code_; }

   private:
    /**
     * @brief Get error message from code
     * @param code Error code
     * @return Error message string
     */
    static std::string getErrorMessage(int code) {
        static const std::unordered_map<int, std::string> error_map = {
            {1001, "PLANNER: Constraints are not given"},
            {1002, "PLANNER: Calculated pose is empty"},
            {1003, "PLANNER: Calculated velocity is empty"},
            {1004, "PLANNER: Calculated acceleration is empty"},
            {1005, "PLANNER: Calculated jerk is empty"},
            {1006, "PLANNER: Final pose is empty"},
            {1007, "PLANNER: Final velocity is empty"},
            {1008, "PLANNER: Value extends boundary"},
            {1009, "PLANNER: Empty input value"}
        };

        auto it = error_map.find(code);
        if (it != error_map.end()) {
            return it->second;
        }
        return "PLANNER: Unknown error";
    }
};

}  // namespace vp

#endif  // VP_PLANNER_EXCEPTION_H
