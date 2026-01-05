#include <pluginlib/class_list_macros.hpp>
#include <order_haziq_base/order_interfaces.hpp>            //Follow the base interface directory


namespace order_haziq_plugins
{
  // ----------------------------
  // Set A: Mid-range (Intel + Nvidia)
  // ----------------------------
  class SetA : public order_haziq_base::OrderSet
  {
  public:
    std::vector<order_haziq_base::Item> get_items() override
    {
      return {
        {"Intel i5 CPU", 850.0, 1},
        {"Nvidia RTX 3060", 1500.0, 1},
        {"16GB RAM", 250.0, 1}
      };
    }
    std::string get_name() override { return "Set A: Intel + Nvidia RTX 3060"; }
  };

  // ----------------------------
  // Set B: Mid-range (AMD + Radeon)
  // ----------------------------
  class SetB : public order_haziq_base::OrderSet
  {
  public:
    std::vector<order_haziq_base::Item> get_items() override
    {
      return {
        {"AMD Ryzen 5 CPU", 800.0, 1},
        {"Radeon RX 6600", 1400.0, 1},
        {"16GB RAM", 250.0, 1}
      };
    }
    std::string get_name() override { return "Set B: AMD + Radeon RX 6600"; }
  };

  // ----------------------------
  // Set C: Budget gaming build
  // ----------------------------
  class SetC : public order_haziq_base::OrderSet
  {
  public:
    std::vector<order_haziq_base::Item> get_items() override
    {
      return {
        {"Intel i3 CPU", 500.0, 1},
        {"GTX 1650", 900.0, 1},
        {"8GB RAM", 150.0, 1}
      };
    }
    std::string get_name() override { return "Set C: Budget Gaming Build"; }
  };

}  // namespace order_haziq_plugins

// Export all sets
PLUGINLIB_EXPORT_CLASS(order_haziq_plugins::SetA, order_haziq_base::OrderSet)
PLUGINLIB_EXPORT_CLASS(order_haziq_plugins::SetB, order_haziq_base::OrderSet)
PLUGINLIB_EXPORT_CLASS(order_haziq_plugins::SetC, order_haziq_base::OrderSet)
