//
// Created by mijiao on 23-7-21.
//

#ifndef DAFUDETECT_CURVESOLVER_H
#define DAFUDETECT_CURVESOLVER_H

#include <ceres/ceres.h>

namespace df {
    struct Ans {
        double a;
        double b;
        double w;
        double phi;
    };

    class CurveSolver {
    private:
        const int N1 = 10;

        struct SPEED_CURVE_FITTING_COST {

            SPEED_CURVE_FITTING_COST(double x, double y, double w0) : _x(x), _y(y), _w0(w0) {}

            template<typename T>
            bool operator()(const T* const ABC, T* residual) const {
                //spd = a * sin(w*t + phi) + b
                //A = a*sin(phi)
                //B = a*cos(phi)
                //C = 2.09 - a
                //error =  y-Acos(w0*x)-Bsin(w0*x)-C
                residual[0] = T(_y) - ABC[0] * ceres::cos(_w0 * T(_x)) - ABC[1] * ceres::sin(_w0 * T(_x)) - ABC[2];
                return true;
            }

            // x, y数据
            const double _x, _y, _w0;
        };

        struct ANGLE_CURVE_FITTING_COST {

            ANGLE_CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

            template<typename T>
            bool operator()(const T* const ABCW, T* residual) const {
                //spd = a * sin(w*t + phi) + b
                //A = a*sin(phi)/w
                //B = a*cos(phi)/w
                //C = 2.09 - a
                //error =  y-Acos(w0*x)-Bsin(w0*x)-C
                residual[0] = T(_y) - ABCW[0] * ceres::sin(ABCW[3] * T(_x)) + ABCW[1] * ceres::cos(ABCW[3] * T(_x)) -
                              ABCW[2] * T(_x);
                return true;
            }

            // x, y数据
            const double _x, _y;
        };

        struct ResultOnce {
            double a;
            double phi;
            double final_cost;
        };

        struct ResultIter {
            double a;
            double phi;
            double w0;
            double w0begin;
            double w0end;
        };

        const double Ae = 0.9125;
        const double Be = 0.9125;
        const double Ce = 1.1175;
        const double We = 1.9420;

        std::vector<std::pair<float, float>> List;

    public:
        explicit CurveSolver(std::vector<std::pair<float, float>> _List) {
            List = _List;
        }

        Ans solve2() {
            double abcw[4] = {Ae, Be, Ce, We};
            ceres::Problem problem;
            for (auto& data: List) {
                problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<ANGLE_CURVE_FITTING_COST, 1, 4>(
                                new ANGLE_CURVE_FITTING_COST(data.first, data.second)
                        ),
                        nullptr,
                        abcw
                );
            }
            ceres::Solver::Options options;
            options.max_num_iterations = 50;
            options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;

            // 设置输出到cout
            options.minimizer_progress_to_stdout = true;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            double a = 2.09 - abcw[2];
            double b = abcw[2];
            double w = abcw[3];
            double phi = atan2(abcw[0], abcw[1]);


#ifdef DEBUG

            // 在屏幕上输出结果
            std::cout << summary.BriefReport() << std::endl;
            std::cout << "a, b, w, phi 为：";

            std::cout << a << " " << b << " " << w << " " << phi;
            std::cout << std::endl << std::endl;
#endif
            return {a, b, w, phi};
        }

        ResultOnce solveOnce(double w0) {

            double abc[3] = {Ae, Be, Ce};
            ceres::Problem problem;
            for (auto& data: List) {
                problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<SPEED_CURVE_FITTING_COST, 1, 3>(
                                new SPEED_CURVE_FITTING_COST(data.first, data.second, w0)
                        ),
                        nullptr,
                        abc
                );
            }
            ceres::Solver::Options options;
            options.max_num_iterations = 50;
            options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;

#ifdef DEBUG_DETAIL
            // 设置输出到cout
            options.minimizer_progress_to_stdout = true;
#endif

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            double a = 2.09 - abc[2];
            double phi = atan2(abc[0], abc[1]);

#ifdef DEBUG_DETAIL

            // 在屏幕上输出结果
            std::cout << summary.BriefReport() << std::endl;
            std::cout << "a, phi 为：";

            std::cout << a << " " << phi;
            std::cout << std::endl << std::endl;
#endif
            return {a, phi, summary.final_cost};
        }

        ResultIter solveIter(double w0begin, double w0end) {
            double bestW0 = 0;
            ResultOnce best = {0, 0, 1e10};
            double range = (w0end - w0begin) / double(N1);
            for (int i = 0; i < N1; i++) {
                double w0 = w0begin + range * i + range / 2;
#ifdef DEBUG_DETAIL
                std::cout << "search: " << w0 << std::endl;
#endif
                auto result = solveOnce(w0);
                if (result.final_cost < best.final_cost) {
                    best = result;
                    bestW0 = w0;
                }
            }
#ifdef DEBUG_DETAIL
            std::cout << "iter end, best w0:" << bestW0 << " cost:" << best.final_cost << std::endl << std::endl
                      << std::endl << std::endl;
#endif

            return {best.a, best.phi, bestW0, bestW0 - range / 2, bestW0 + range / 2};
        }

        Ans solve() {
            double wRange[] = {1.884, 2.000};
            auto result = solveIter(wRange[0], wRange[1]);
            auto result2 = solveIter(result.w0begin, result.w0end);

#ifdef DEBUG
            std::cout << "Best answer:" << std::endl
                      << "a:" << result2.a << std::endl
                      << "b:" << (2.09 - result2.a) << std::endl
                      << "w:" << result2.w0 << std::endl
                      << "phi:" << result2.phi << std::endl;
#endif
            return {result2.a, 2.09 - result2.a, result2.w0, result2.phi};
        }

    };
}

#endif //DAFUDETECT_CURVESOLVER_H
