#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "instructions.h"

void fillInstructionSet(Instructions::Set& set) {
        auto minus = [](double a, double b) -> double { return a - b; };
        auto add = [](double a, double b) -> double { return a + b; };
        auto mult = [](double a, double b) -> double { return a * b; };
        auto div = [](double a, double b) -> double { return a / b; };
        auto max = [](double a, double b) -> double { return std::max(a, b); };
        auto ln = [](double a) -> double { return std::log(a); };
        auto exp = [](double a) -> double { return std::exp(a); };
        auto cos = [](double a) -> double { return std::cos(a); };
        auto sin = [](double a) -> double { return std::sin(a); };
        auto tan = [](double a) -> double { return std::tan(a); };

        auto modulo = [](double a, double b) -> double { return  fmod(a, b);};

        /*auto min = [](double a, double b) -> double { return std::min(a, b); };
        auto absolute = [](double a) -> double {return abs(a); };
        auto reciprocal = [](double a) -> double {return 1/a; };
        auto arcsin = [](double a) -> double {return std::asin(a); };
        auto arccos = [](double a) -> double {return std::acos(a); };
        auto arctan = [](double a) -> double {return std::atan(a); };
        auto heavySide = [](double a) -> double {return (a >= 0) ? 1:0; };
        auto pow = [](double a) -> double {return std::pow(a, 2); };
        auto cube = [](double a) -> double {return std::pow(a, 3); };
        auto sqrt = [](double a) -> double {return std::sqrt(a); };
        auto tanh = [](double a) -> double {return std::tanh(a); }; */

        set.add(*(new Instructions::LambdaInstruction<double, double>(minus, "$0 = $1 - $2;")));
        set.add(*(new Instructions::LambdaInstruction<double, double>(add, "$0 = $1 + $2;")));
        set.add(*(new Instructions::LambdaInstruction<double, double>(mult, "$0 = $1 * $2;")));
        set.add(*(new Instructions::LambdaInstruction<double, double>(div, "$0 = $1 / $2;")));
        set.add(*(new Instructions::LambdaInstruction<double, double>(max, "$0 = (($1) < ($2)) ? ($2) : ($1);")));
        set.add(*(new Instructions::LambdaInstruction<double>(exp, "$0 = exp($1);")));
        set.add(*(new Instructions::LambdaInstruction<double>(ln, "$0 = log($1);")));
        set.add(*(new Instructions::LambdaInstruction<double>(cos, "$0 = cos($1);")));
        set.add(*(new Instructions::LambdaInstruction<double>(sin, "$0 = sin($1);")));
        set.add(*(new Instructions::LambdaInstruction<double>(tan, "$0 = tan($1);")));

        set.add(*(new Instructions::LambdaInstruction<double, double>(modulo)));

        /*set.add(*(new Instructions::LambdaInstruction<double, double>(min)));
        set.add(*(new Instructions::LambdaInstruction<double>(absolute)));
        set.add(*(new Instructions::LambdaInstruction<double>(reciprocal)));
        set.add(*(new Instructions::LambdaInstruction<double>(arcsin)));
        set.add(*(new Instructions::LambdaInstruction<double>(arccos)));
        set.add(*(new Instructions::LambdaInstruction<double>(arctan)));
        set.add(*(new Instructions::LambdaInstruction<double>(heavySide)));
        set.add(*(new Instructions::LambdaInstruction<double>(pow)));
        set.add(*(new Instructions::LambdaInstruction<double>(cube)));
        set.add(*(new Instructions::LambdaInstruction<double>(sqrt)));
        set.add(*(new Instructions::LambdaInstruction<double>(tanh)));*/

        if(true ){
            auto minusC = [](double a, double b, Data::Constant c, Data::Constant d) -> double { return a*(double)c - b*(double)d; };
            auto addC = [](double a, double b, Data::Constant c, Data::Constant d) -> double { return a*(double)c + b*(double)d; };
            auto multC = [](double a, double b, Data::Constant c) -> double { return a * b*(double)c; };
            auto divC = [](double a, double b, Data::Constant c) -> double { return a / b*(double)c; };
            //auto max = [](double a, double b) -> double { return std::max(a, b); };
            auto lnC = [](double a, Data::Constant c) -> double { return std::log(a*(double)c); };
            auto expC = [](double a, Data::Constant c) -> double { return std::exp(a*(double)c); };
            auto cosC = [](double a, Data::Constant c) -> double { return std::cos(a*(double)c); };
            auto sinC = [](double a, Data::Constant c) -> double { return std::sin(a*(double)c); };
            auto tanC = [](double a, Data::Constant c) -> double { return std::tan(a*(double)c); };
            //auto pi = [](double a, Data::Constant c) -> double { return M_PI*(double)c; };
            //auto multByConst = [](double a, Data::Constant c) -> double { return a * (double)c / 10.0; };

            set.add(*(new Instructions::LambdaInstruction<double, double, Data::Constant, Data::Constant>(minusC, "$0 = $1 - $2;")));
            set.add(*(new Instructions::LambdaInstruction<double, double, Data::Constant, Data::Constant>(addC, "$0 = $1 + $2;")));
            set.add(*(new Instructions::LambdaInstruction<double, double, Data::Constant>(multC, "$0 = $1 * $2;")));
            set.add(*(new Instructions::LambdaInstruction<double, double, Data::Constant>(divC, "$0 = $1 / $2;")));
            //set.add(*(new Instructions::LambdaInstruction<double, double>(max, "$0 = (($1) < ($2)) ? ($2) : ($1);")));
            set.add(*(new Instructions::LambdaInstruction<double, Data::Constant>(expC, "$0 = exp($1);")));
            set.add(*(new Instructions::LambdaInstruction<double, Data::Constant>(lnC, "$0 = log($1);")));
            set.add(*(new Instructions::LambdaInstruction<double, Data::Constant>(cosC, "$0 = cos($1);")));
            set.add(*(new Instructions::LambdaInstruction<double, Data::Constant>(sinC, "$0 = sin($1);")));
            set.add(*(new Instructions::LambdaInstruction<double, Data::Constant>(tanC, "$0 = tan($1);")));
        }

}