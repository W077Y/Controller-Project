#include "tst_data_calc_SISO_KalmanIntegralController.hpp"

#include <controller.hpp>
#include <iostream>
#include <ut_catch.hpp>

TEST_CASE()
{
  using T = controller::calculator::SISO_KalmanIntegralController<float, 3>;

  T calc{ test_data::parameter::param };

  for (std::size_t ch_r = 0; ch_r < 2; ch_r++)
  {
    for (std::size_t i = 0; i < test_data::parameter::y_meas.number_of_columns; i++)
    {
      calc.correction_step(test_data::parameter::y_meas(0, i));

      auto const x = calc.get_x();
      for (std::size_t j = 0; j < x.number_of_rows; j++)
      {
        REQUIRE(x(j, 0) == Approx(test_data::parameter::x(j, i)).margin(1E-3));
      }

      auto const u_star = calc.calculate_u_star();
      REQUIRE(u_star == Approx(test_data::parameter::u_star(0, i)).margin(1E-3));

      auto const e = test_data::parameter::r(0, i) - calc.calculate_y_int();
      calc.update_observer(test_data::parameter::u(0, i));
      calc.update_integrator(test_data::parameter::u_star(0, i), test_data::parameter::u_dash(0, i), e);
    }

    calc.reset_observer();
    calc.reset_integrator();
  }
}
