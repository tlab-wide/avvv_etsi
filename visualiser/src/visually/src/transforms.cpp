#include <visually/transforms.hpp>

namespace transforms
{

rviz_visual_tools::Colors getRvizColour(
    const std::string& colour)
{
    using rviz_visual_tools::Colors;

    if (colour == "black")
        return Colors::BLACK;
    if (colour == "brown")
        return Colors::BROWN;
    if (colour == "blue")
        return Colors::BLUE;
    if (colour == "cyan")
        return Colors::CYAN;
    if (colour == "grey")
        return Colors::GREY;
    if (colour == "dark_grey")
        return Colors::DARK_GREY;
    if (colour == "green")
        return Colors::GREEN;
    if (colour == "lime_green")
        return Colors::LIME_GREEN;
    if (colour == "magenta")
        return Colors::MAGENTA;
    if (colour == "orange")
        return Colors::ORANGE;
    if (colour == "purple")
        return Colors::PURPLE;
    if (colour == "red")
        return Colors::RED;
    if (colour == "pink")
        return Colors::PINK;
    if (colour == "white")
        return Colors::WHITE;
    if (colour == "yellow")
        return Colors::YELLOW;
    if (colour == "translucent")
        return Colors::TRANSLUCENT;
    if (colour == "translucent_light")
        return Colors::TRANSLUCENT_LIGHT;
    if (colour == "translucent_dark")
        return Colors::TRANSLUCENT_DARK;
    if (colour == "rand")
        return Colors::RAND;
    if (colour == "clear")
        return Colors::CLEAR;
    return Colors::DEFAULT;

}

void getGeometryPose(
    const double position[3]
    , const double orientation[3]
    , geometry_msgs::msg::Pose& pose)
{
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];

    // Using the tf2 package to conveniently convert orientation to quaternion
    tf2::Quaternion q;
    q.setRPY(orientation[0], orientation[1], orientation[2]);
    
    geometry_msgs::msg::Quaternion quaternion;
    pose.orientation.w = q.getW();
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
}

double dist(
    const geometry_msgs::msg::Point& point1
    , const geometry_msgs::msg::Point& point2)
{
    return sqrt(
        pow(point1.x - point2.x, 2) +
        pow(point1.y - point2.y, 2) +
        pow(point1.z - point2.z, 2)
    );
}

double dist(
    const Eigen::Vector3d& point1
    , const Eigen::Vector3d& point2)
{
    return sqrt(
        pow(point1.x() - point2.x(), 2) +
        pow(point1.y() - point2.y(), 2) +
        pow(point1.z() - point2.z(), 2)
    );
}

void linspace(
    double array[]
    , double start_point
    , int num_packets
    , double step
    , double progress)
{
    array[0] = start_point + progress;
    for (int i{ 1 }; i < num_packets; ++i)
        array[i] = array[i - 1] + step;
}

void multiplyMatrices(
    double result[3][3]
    , const double matrix1[3][3]
    , const double matrix2[3][3])
{
    for(short i{ 0 }; i < 3; ++i)
        for(short j = 0; j < 3; ++j) {
            result[i][j] = 0;
            for(short k = 0; k < 3; ++k)
                result[i][j] += matrix1[i][k] * matrix2[k][j];
        }
}

void multiplyMatrices(
    double result[4][1]
    , const double matrix1[4][4]
    , const double matrix2[4][1])
{
    for(int i{ 0 }; i < 4; ++i) {
        result[i][0] = 0;
        for(int k = 0; k < 4; ++k)
            result[i][0] += matrix1[i][k] * matrix2[k][0];
    }
}

void getTransformMatrix(
    double result[4][4]
    , const double translation[3]
    , const double rotation[3])
{
    double rotx[3][3]{
        {1      , 0                         , 0                         },
        {0      , std::cos(rotation[0])     , -std::sin(rotation[0])    },
        {0      , std::sin(rotation[0])     , std::cos(rotation[0])     }
    };

    double roty[3][3]{
        {std::cos(rotation[1])      , 0     , std::sin(rotation[1])     },
        {0                          , 1     , 0                         },
        {-std::sin(rotation[1])     , 0     , std::cos(rotation[1])     }
    };

    double rotz[3][3]{
        {std::cos(rotation[2])      , -std::sin(rotation[2])    , 0     },
        {std::sin(rotation[2])      , std::cos(rotation[2])     , 0     },
        {0                          , 0                         , 1     }
    };

    double rotzy[3][3];

    multiplyMatrices(rotzy, rotz, roty);

    double rot[3][3];
    multiplyMatrices(rot, rotzy, rotx);

    for(int i{ 0 }; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            result[i][j] = rot[i][j];
    
    for(int i{ 0 }; i < 3; ++i) {
        result[i][3] = translation[i];
        result[3][i] = 0;
    }

    result[3][3] = 1;
}

void transformPoint(
    double result[3]
    , const double point[3]
    , const double transform_matrix[4][4])
{
    double transformed_point[4][1];
    double original_point[4][1]{
        {point[0]},
        {point[1]},
        {point[2]},
        {1}
    };
    multiplyMatrices(
        transformed_point
        , transform_matrix
        , original_point);
    for(int i{ 0 }; i < 3; ++i)
        result[i] = transformed_point[i][0];
}

void pose2Array(
    const geometry_msgs::msg::Pose& pose
    , double position[3]
    , double orientation[3])
{
    // Convert pose to straightforward arrays
    position[0] = pose.position.x;
    position[1] = pose.position.y;
    position[2] = pose.position.z;
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(orientation[0], orientation[1], orientation[2]);
}

void toMgrs(
    double lon
    , double lat
    , double& x
    , double& y)
{
    static int zone;
    static bool northp;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
    x = x - (static_cast<int>(x) / 100'000) * 100'000;
    y = y - (static_cast<int>(y) / 100'000) * 100'000;
}

}