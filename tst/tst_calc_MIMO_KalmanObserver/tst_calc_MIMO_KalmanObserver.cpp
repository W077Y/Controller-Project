#include "tst_data_calc_MIMO_KalmanObserver.hpp"

#include <controller.hpp>
#include <iostream>
#include <ut_catch.hpp>

TEST_CASE()
{
  using T = controller::calculator::MIMO_KalmanObserver<float, 5, 2, 2>;

  T calc{ test_data::parameter::param };

  for (std::size_t i = 0; i < test_data::parameter::y_meas.number_of_columns; i++)
  {
    calc.correction_step({ test_data::parameter::y_meas(0, i), test_data::parameter::y_meas(1, i) });

    auto const x = calc.get_x();
    auto const y = calc.get_y();
    for (std::size_t j = 0; j < x.number_of_rows; j++)
    {
      REQUIRE(x(j, 0) == Approx(test_data::parameter::x(j, i)).margin(1E-5));
    }
    for (std::size_t j = 0; j < y.number_of_rows; j++)
    {
      REQUIRE(y(j, 0) == Approx(test_data::parameter::y(j, i)).margin(1E-5));
    }

    calc.update_step({ test_data::parameter::u(0, i), test_data::parameter::u(1, i) });
  }
}
