<launch>
  <node pkg="joy_node" name="joystick_node.py" type="joystick_node.py" output="screen" />
  <node pkg="rasberry" name="raspberry.py" type="raspberry_node.py" output="screen" />
  <node pkg="road_to_autonomy" name="computer_node.py" type="stm32_node.py" output="screen" />
</launch>
