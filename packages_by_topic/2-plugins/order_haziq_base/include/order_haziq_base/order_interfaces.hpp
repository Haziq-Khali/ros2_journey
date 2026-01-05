#ifndef ORDER_HAZIQ_BASE__ORDER_INTERFACES_HPP_
#define ORDER_HAZIQ_BASE__ORDER_INTERFACES_HPP_

#include <string>
#include <vector>

namespace order_haziq_base
{
  // Shared item struct
  struct Item
  {
    std::string name;
    double price;
    int quantity;
  };

  // ----------------------------
  // Calculator interface
  // ----------------------------
  class OrderCalculator
  {
  public:
    virtual ~OrderCalculator() = default;

    // Input: list of items, return total cost
    virtual double calculate_total(const std::vector<Item>& items) = 0;

    // Identify which calculator this is
    virtual std::string get_name() = 0;
  };

  // ----------------------------
  // Set interface
  // ----------------------------
  class OrderSet
  {
  public:
    virtual ~OrderSet() = default;

    // Return list of items in this set
    virtual std::vector<Item> get_items() = 0;

    // Identify which set this is
    virtual std::string get_name() = 0;
  };

}  // namespace order_haziq_base

#endif  // ORDER_HAZIQ_BASE__ORDER_INTERFACES_HPP_
