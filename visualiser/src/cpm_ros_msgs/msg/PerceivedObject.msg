# A Perceived Object Container can be added for every object that has been perceived by an ITS-S. It provides
# information about the detected object with respect to the disseminating station. Classifications and positions matched to
# road data can also be provided. This container type is only added if objects have been detected according to the
# inclusion rules defined in clause ( ETSI TR 103 562 V2.1.1 (2019-12) => 6.1 )


# Identifier assigned to a detected object which remains constant as long as the object is
# perceived by the disseminating ITS-S. Numbers are assigned in an increasing round-robin
# fashion. When the last identifier in the allowed range has been used, the first counter for the
# identifier starts from the beginning of the range again. ( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.2 )

string object_id


# Absolute distance to detected object from the ITS-S's reference point in x-direction at
# the time of measurement. For a vehicle, it is according to the coordinate system
# provided by ISO 8855 [i.2]. For a RSU, it is according to a coordinate system where y
# corresponds to the North direction, x to the East direction, and z to the vertical direction,
# as outlined in Figure 45 ( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.7 )

float64 x_distance


# Absolute distance to detected object from the ITS-S's reference point in y-direction at the
# time of measurement. For a vehicle, it is according to the coordinate system provided by
# ISO 8855 [i.2]. For a RSU, it is according to a coordinate system where y corresponds to the
# North direction, x to the East direction, and z to the vertical direction, as outlined in
# Figure 45. ( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.8 )

float64 y_distance


# Absolute distance to detected object from the ITS-S's reference point in z-direction at the
# time of measurement. For a vehicle, it is according to the coordinate system provided by
# ISO 8855 [i.2]. For a RSU, it is according to a coordinate system where y corresponds to the
# North direction, x to the East direction, and z to the vertical direction, as outlined in
# Figure 45 . ( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.9 )

float64 z_distance 0.0


# Relative speed of detected object from the ITS-S's reference point in x-direction at the time
# of measurement. For a vehicle, it is according to the coordinate system provided by
# ISO 8855 [i.2]. For a RSU, it is according to a coordinate system where y corresponds to the
# North direction, x to the East direction, and z to the vertical direction, as outlined in
# Figure 45. ( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.10 )

float64 x_speed


# Relative speed of detected object from the ITS-S's reference point in y-direction at the time
# of measurement. For a vehicle, it is according to the coordinate system provided by
# ISO 8855 [i.2]. For a RSU, it is according to a coordinate system where y corresponds to the
# North direction, x to the East direction, and z to the vertical direction, as outlined in
# Figure 45. ( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.11 )

float64 y_speed


# Relative speed of detected object from the ITS-S's reference point in z-direction at the time
# of measurement. For a vehicle, it is according to the coordinate system provided by
# ISO 8855 [i.2]. For a RSU, it is according to a coordinate system where y corresponds to the
# North direction, x to the East direction, and z to the vertical direction, as outlined in
# Figure 45. ( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.12 )

float64 z_speed 0.0


float64 roll_angle 0.0
float64 pitch_angle 0.0


# Relative yaw angle of object from the ITS-S's reference point. For a vehicle, it is according to
# the vehicle x direction in the coordinate system provided by ISO 8855 [i.2]. For a RSU, it is
# according to the x direction in a coordinate system where y corresponds to the North
# direction, x to the East direction, and z to the vertical direction, as outlined in Figure 45.
# The angle is measured with positive values considering the object orientation turning
# counter-clockwise starting from the x-direction. 3601 should be set if the value is unavailable
# The yaw angle confidence is described with a predefined confidence level of 95 % for the
# component.( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.16 )

float64 yaw_angle


# First dimension of object as provided by the sensor or environment model. This dimension is
# always contained in the plane which is perpendicular to the direction of the angle indicated
# by the yawAngle and which contains the object reference point.( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.17 )

float64 planar_object_dimension1


# Second dimension of the object as provided by the sensor environment model. This
# dimension is contained in the plane which contains the direction of the angle indicated by the
# yawAngle and the object reference point.( ETSI TR 103 562 V2.1.1 (2019-12) => B.5.18 )

float64 planar_object_dimension2


# Vertical dimension of object as provided by the sensor or object model.

float64 vertical_object_dimension
