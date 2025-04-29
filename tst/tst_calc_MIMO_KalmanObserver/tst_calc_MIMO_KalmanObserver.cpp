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
    calc.correction_step(test_data::parameter::y_meas(0, i));

    auto const x = calc.get_x();
    auto const y = calc.get_y();
    for (std::size_t j = 0; j < x.number_of_rows; j++)
    {
      REQUIRE(x(j, 0) == Approx(test_data::parameter::x(j, i)).margin(1E-3));
    }
    REQUIRE(y(0, 0) == Approx(test_data::parameter::y(0, i)).margin(1E-3));

    calc.update_step({ test_data::parameter::u(0, i), test_data::parameter::u(1, i) });
  }
}
