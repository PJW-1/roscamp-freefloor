cmake_minimum_required(VERSION 3.8)
project(fleet_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/RobotState.msg"
      "msg/ArmState.msg"
      "msg/ItemArray.msg"
      "msg/OrderItem.msg"
      "msg/Order.msg"
      "msg/OrderBoard.msg"
      "msg/Assignment.msg"
      "msg/AssignmentArray.msg"
      "msg/ZoneState.msg"
      "msg/ZoneStateArray.msg"
      "msg/FleetEvent.msg"
      "action/MoveTo.action"
      "action/Dock.action"
      "action/FillBasket.action"
      "action/SwapBasket.action"
      "srv/SetRunMode.srv"
      "srv/ReportOrder.srv"
      "srv/SubmitOrder.srv"
      "srv/ConfirmClear.srv"
      DEPENDENCIES geometry_msgs builtin_interfaces)
install(FILES contract.yaml DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
