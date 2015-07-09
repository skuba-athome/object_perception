# The information for the plane that has been detected
Table table

# The information for the objects on the plane
string[] names 
sensor_msgs/PointCloud[] clusters
geometry_msgs/Point[] centriods
shape_msgs/SolidPrimitive[] solid_boxes

# Whether the detection has succeeded or failed
int32 NO_CLOUD_RECEIVED = 1
int32 NO_TABLE = 2
int32 OTHER_ERROR = 3
int32 SUCCESS = 4
int32 NO_OBJECT = 5
int32 result