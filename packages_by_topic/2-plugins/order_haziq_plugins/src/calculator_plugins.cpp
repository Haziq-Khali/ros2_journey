#include <pluginlib/class_list_macros.hpp>
#include <order_haziq_base/order_interfaces.hpp>            //Follow the base interface directory

namespace order_haziq_plugins
{
  // ----------------------------
  // Basic Calculator (no discount)
  // ----------------------------
  class BasicCalculator : public order_haziq_base::OrderCalculator
  {
  public:
    double calculate_total(const std::vector<order_haziq_base::Item>& items) override
    {
      double total = 0.0;
      for (auto &item : items) {
        total += item.price * item.quantity;
      }
      return total;
    }

    std::string get_name() override { return "Basic Calculator"; }
  };

  // ----------------------------
  // Discount Calculator (10% off total)
  // ----------------------------
  class DiscountCalculator : public order_haziq_base::OrderCalculator
  {
  public:
    double calculate_total(const std::vector<order_haziq_base::Item>& items) override
    {
      double total = 0.0;
      for (auto &item : items) {
        total += item.price * item.quantity;
      }
      return total * 0.9;  // 10% discount
    }

    std::string get_name() override { return "Discount Calculator (10% off)"; }
  };

}  // namespace order_haziq_plugins

// Export both calculators
PLUGINLIB_EXPORT_CLASS(order_haziq_plugins::BasicCalculator, order_haziq_base::OrderCalculator)
PLUGINLIB_EXPORT_CLASS(order_haziq_plugins::DiscountCalculator, order_haziq_base::OrderCalculator)
