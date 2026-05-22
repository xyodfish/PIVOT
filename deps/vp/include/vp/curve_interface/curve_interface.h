/**
 * @file curve_interface.h
 * @brief Velocity Planning Module - Curve Interface
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_CURVE_INTERFACE_H
#define VP_CURVE_INTERFACE_H

namespace vp {

/**
 * @brief Abstract interface for trajectory curves
 */
class CurveInterface {
   public:
    CurveInterface() = default;
    virtual ~CurveInterface() = default;

    /**
     * @brief Evaluate the curve at given parameter
     * @param param Parameter value (usually time)
     * @param order Derivative order (0=position, 1=velocity, 2=acceleration, 3=jerk)
     * @return Evaluated value
     */
    virtual double evaluate(const double param, const unsigned int order = 0) const = 0;

    /**
     * @brief Get the parameter length of the curve
     * @return Parameter length
     */
    virtual double paramLength() const = 0;
};

}  // namespace vp

#endif  // VP_CURVE_INTERFACE_H
