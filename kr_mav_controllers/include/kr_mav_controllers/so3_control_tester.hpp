#include <kr_mav_msgs/Corrections.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_mav_msgs/SO3Command.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>

class SO3ControlTester
{
 public:
  SO3ControlTester();
  void so3CommandCallback(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void publish_single_position_command();
  void publish_enable_motors(bool flag);
  void populate_position_cmd_vector(int x_gain, int y_gain, int z_gain, uint8_t use_msg_gains_flags, float yaw_dot);
  void publish_position_command(int index);
  void populate_odom_msgs();
  void publish_odom_msg(int index);
  void reset_so3_cmd_pointer();
  void publish_corrections(float kf_correction, float (&angle_corrections)[2]);
  bool is_so3_cmd_publisher_active();
  kr_mav_msgs::SO3Command::Ptr so3_cmd_;
  bool so3_command_received_ = false;
  std::mutex mutex;

 private:
  ros::NodeHandle nh_;
  ros::Publisher position_cmd_pub_, enable_motors_pub_, odom_pub_, corrections_pub_;
  ros::Subscriber so3_cmd_sub_;
  std::vector<kr_mav_msgs::PositionCommand> position_cmds;
  std::vector<nav_msgs::Odometry> odom_msgs;

  std::array<float, 11> angles = {0.0, 0.314, 0.628, 0.942, 1.257, 1.571, 1.885, 2.199, 2.513, 2.827, 3.142};
};

SO3ControlTester::SO3ControlTester() : nh_("")
{
  position_cmd_pub_ = nh_.advertise<kr_mav_msgs::PositionCommand>("position_cmd", 5, true);
  so3_cmd_sub_ = nh_.subscribe("so3_cmd", 5, &SO3ControlTester::so3CommandCallback, this);
  enable_motors_pub_ = nh_.advertise<std_msgs::Bool>("motors", 5, true);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5, true);
  corrections_pub_ = nh_.advertise<kr_mav_msgs::Corrections>("corrections", 5, true);
}

/*
 * @brief Function to check if the so_cmd publisher is active on the nodelet side.
 *        Helps in checking if the nodelet is active.
 */
bool SO3ControlTester::is_so3_cmd_publisher_active()
{
  bool flag = false;
  if(so3_cmd_sub_.getNumPublishers() > 0)
    flag = true;
  else
  {
    ros::Duration(1.0).sleep();
    if(so3_cmd_sub_.getNumPublishers() > 0)
      flag = true;
    else
      flag = false;
  }
  return flag;
}

/*
 * @brief Callback function to hand so3 command messages from nodelet.
 *
 * @param msg Type: kr_mav_msgs::SO3Command::ConstPtr
 */
void SO3ControlTester::so3CommandCallback(const kr_mav_msgs::SO3Command::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  so3_command_received_ = true;
  if(!so3_cmd_)
  {
    so3_cmd_ = boost::make_shared<kr_mav_msgs::SO3Command>(*msg);
  }
  else
  {
    so3_cmd_.reset();
    so3_cmd_ = boost::make_shared<kr_mav_msgs::SO3Command>(*msg);
  }
}

/*
 * @brief Function to reset the pointer that points to the so3 command msg.
 */
void SO3ControlTester::reset_so3_cmd_pointer()
{
  std::lock_guard<std::mutex> lock(mutex);
  if(so3_cmd_)
    so3_cmd_.reset();
  so3_command_received_ = false;
}

/*
 * @brief Function to publish a single position command over the topic `position_cmd`
 */
void SO3ControlTester::publish_single_position_command()
{
  kr_mav_msgs::PositionCommand cmd;
  cmd.position.x = 1.0;
  cmd.position.y = 0.2;
  cmd.position.z = 5.0;
  cmd.velocity.x = 0.1;
  cmd.velocity.y = 0.1;
  cmd.velocity.z = 0.1;
  cmd.acceleration.x = 0.01;
  cmd.acceleration.y = 0.01;
  cmd.acceleration.z = 0.01;
  cmd.jerk.x = 0.001;
  cmd.jerk.y = 0.001;
  cmd.jerk.z = 0.001;
  cmd.yaw = 1.0;
  cmd.yaw_dot = 0.1;
  cmd.kx[0] = 1.0;
  cmd.kx[1] = 1.0;
  cmd.kx[2] = 1.0;
  cmd.kv[0] = 0.1;
  cmd.kv[1] = 0.1;
  cmd.kv[2] = 0.1;
  cmd.use_msg_gains_flags = 0;
  position_cmd_pub_.publish(cmd);
}

/*
 * @brief Function to publish message over the topic `motors`
 *
 * @param flag boolean flag message to enable/disable motors
 */
void SO3ControlTester::publish_enable_motors(bool flag)
{
  std_msgs::Bool msg;
  msg.data = flag;
  enable_motors_pub_.publish(msg);
}

/*
 * @brief Function that populates the a vector of `kr_mav_messages::PositionCommand`
 *        Generates position commands based on sine and cosine functions
 *
 * @param x_gain int, max x-position
 * @param y_gain int, max y-position
 * @param z_gain int, max z_position
 * @param use_msg_gains_flags uint8_t, indicates which gains to use from the message
 * @param yaw_dot float, desired yaw_dot
 */
void SO3ControlTester::populate_position_cmd_vector(int x_gain, int y_gain, int z_gain, uint8_t use_msg_gains_flags,
                                                    float yaw_dot)
{
  position_cmds.clear();
  int angles_arr_size = angles.size();
  for(int i = 0; i < angles_arr_size; i++)
  {
    float x_sin = x_gain * std::sin(angles[i] + M_PI_2);
    float x_cos = x_gain * std::cos(angles[i] + M_PI_2);
    float y_sin = y_gain * std::sin(angles[i] + M_PI);
    float y_cos = y_gain * std::cos(angles[i] + M_PI);
    float z_sin = z_gain * std::sin(angles[i]);
    float z_cos = z_gain * std::cos(angles[i]);

    kr_mav_msgs::PositionCommand cmd;
    cmd.position.x = x_sin;
    cmd.position.y = y_sin;
    cmd.position.z = z_sin;
    cmd.velocity.x = x_cos;
    cmd.velocity.y = y_cos;
    cmd.velocity.z = z_cos;
    cmd.acceleration.x = (-1) * x_sin;
    cmd.acceleration.y = (-1) * y_sin;
    cmd.acceleration.z = (-1) * z_sin;
    cmd.jerk.x = (-1) * x_cos;
    cmd.jerk.y = (-1) * y_cos;
    cmd.jerk.z = (-1) * z_cos;
    cmd.yaw = angles[i];
    cmd.yaw_dot = yaw_dot;
    cmd.kx[0] = 1.0;
    cmd.kx[1] = 1.0;
    cmd.kx[2] = 1.0;
    cmd.kv[0] = 0.1;
    cmd.kv[1] = 0.1;
    cmd.kv[2] = 0.1;
    cmd.use_msg_gains_flags = use_msg_gains_flags;
    position_cmds.push_back(cmd);
  }
}

/*
 * @brief Function to publish position command indexed from the vector of `kr_mav_msgs::PositionCommand` msgs
 *
 * @param index int
 */
void SO3ControlTester::publish_position_command(int index)
{
  int position_cmds_vector_size = position_cmds.size();
  if(index < position_cmds_vector_size)
  {
    position_cmd_pub_.publish(position_cmds[index]);
  }
}

/*
 * @brief Function to populate a vector with 3 odom messages of type `nav_msgs::Odometry`
 */
void SO3ControlTester::populate_odom_msgs()
{
  odom_msgs.clear();

  nav_msgs::Odometry msg;
  msg.header.frame_id = "quadrotor";
  msg.pose.pose.position.x = 0.0;
  msg.pose.pose.position.y = 0.0;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  msg.pose.pose.orientation.w = 1.0;
  msg.twist.twist.linear.x = 0.0;
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = 0.0;
  odom_msgs.push_back(msg);

  msg.header.frame_id = "quadrotor";
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 0.5;
  msg.pose.pose.position.z = 2.0;
  msg.pose.pose.orientation.x = 0.0937556;
  msg.pose.pose.orientation.y = 0.0787129;
  msg.pose.pose.orientation.z = 0.0937556;
  msg.pose.pose.orientation.w = 0.9880405;
  msg.twist.twist.linear.x = 0.01;
  msg.twist.twist.linear.y = 0.05;
  msg.twist.twist.linear.z = 0.1;
  msg.twist.twist.angular.x = 0.02;
  msg.twist.twist.angular.y = 0.02;
  msg.twist.twist.angular.z = 0.05;
  odom_msgs.push_back(msg);

  msg.header.frame_id = "quadrotor";
  msg.pose.pose.position.x = 0.5;
  msg.pose.pose.position.y = 1.0;
  msg.pose.pose.position.z = 1.0;
  msg.pose.pose.orientation.x = 0.0865617;
  msg.pose.pose.orientation.y = -0.0865617;
  msg.pose.pose.orientation.z = -0.0075499;
  msg.pose.pose.orientation.w = 0.9924501;
  msg.twist.twist.linear.x = 0.05;
  msg.twist.twist.linear.y = 0.01;
  msg.twist.twist.linear.z = 0.2;
  msg.twist.twist.angular.x = 0.04;
  msg.twist.twist.angular.y = 0.04;
  msg.twist.twist.angular.z = 0.1;
  odom_msgs.push_back(msg);
}

/*
 * @brief Function to publish odom message index from a vector of `nav_msgs::Odometry` msgs
 *
 * @param index int
 */
void SO3ControlTester::publish_odom_msg(int index)
{
  int odom_vector_size = odom_msgs.size();
  if(index < odom_vector_size)
  {
    odom_pub_.publish(odom_msgs[index]);
  }
}

/*
 * @brief Function to publish corrections message of type `kr_mav_msgs::Corrections`
 *
 * @param kf_correction float
 * @param angle_corrections float[2]
 */
void SO3ControlTester::publish_corrections(float kf_correction, float (&angle_corrections)[2])
{
  kr_mav_msgs::Corrections msg;
  msg.kf_correction = kf_correction;
  msg.angle_corrections[0] = angle_corrections[0];
  msg.angle_corrections[1] = angle_corrections[1];
  corrections_pub_.publish(msg);
}

/*
 * @brief Struct to store reference data for Test4
 */
struct Test4Data
{
  float force_x = 3.945000171661377;
  float force_y = 0.9850000143051147;
  float force_z = 31.209999084472656;
  float orientation_x = 0.01621658354997635;
  float orientation_y = 0.06266678869724274;
  float orientation_z = 0.476580411195755;
  float orientation_w = 0.8767445683479309;
  float angular_velocity_x = 0.0019372408278286457;
  float angular_velocity_y = 0.014631535857915878;
  float angular_velocity_z = 0.09864499419927597;
  float current_yaw = 0.0;
  float kf_correction = 0.0;
  float angle_corrections[2] = {0.0, 0.0};
  bool enable_motors = true;
  bool use_external_yaw = true;
  float kR[3] = {1.5, 1.5, 1.0};
  float kOm[3] = {0.12999999523162842, 0.12999999523162842, 0.10000000149011612};
};

/*
 * @brief Struct to store reference data for Test5
 */
struct Test5Data
{
  float force_x[11] = {3.200000047683716,   2.3022608757019043,  1.1793856620788574,  -0.058816105127334595,
                       -1.2950551509857178, -2.400651693344116,  -3.2714920043945312, -3.822418212890625,
                       -3.999556064605713,  -3.7855801582336426, -3.1990225315093994};
  float force_y[11] = {-2.4000000953674316,  -3.2710256576538086, -3.8221793174743652, -3.999568462371826,
                       -3.7845540046691895,  -3.1995112895965576, -2.30159592628479,   -1.1786112785339355,
                       0.059632182121276855, 1.2920401096343994,  2.40130352973938};
  float force_z[11] = {10.905000686645508, 13.514970779418945, 15.283012390136719, 16.036128997802734,
                       15.698030471801758, 14.303777694702148, 11.990468978881836, 8.984258651733398,
                       5.579117298126221,  2.1081130504608154, -1.0988283157348633};
  float orientation_x[11] = {0.10281641036272049,  0.12832938134670258,  0.1276189535856247,   0.1074145957827568,
                             0.07156907767057419,  0.020389312878251076, -0.05070211738348007, -0.15126314759254456,
                             -0.29631441831588745, -0.50934898853302,    0.7754440307617188};
  float orientation_y[11] = {0.14146201312541962,  0.06395066529512405,  -0.0025313780643045902, -0.057677000761032104,
                             -0.10178329795598984, -0.13438037037849426, -0.15207025408744812,   -0.142638698220253,
                             -0.07645609974861145, 0.08627571910619736,  -0.1764468401670456};
  float orientation_z[11] = {-0.014773921109735966, 0.1488552689552307, 0.3111719787120819, 0.45663392543792725,
                             0.5831654667854309,    0.6942919492721558, 0.7941436171531677, 0.8777838945388794,
                             0.918185293674469,     0.8451351523399353, -0.5535274147987366};
  float orientation_w[11] = {0.9844790697097778,  0.9784088730812073, 0.9417425394058228,  0.8812609910964966,
                             0.802767813205719,   0.7067424654960632, 0.5862080454826355,  0.4315890669822693,
                             0.25157108902931213, 0.1373560130596161, -0.24730625748634338};
  float angular_velocity_x[11] = {0.10910380631685257,  0.008616520091891289, -0.05797356739640236,
                                  -0.10991425812244415, -0.1590983271598816,  -0.21264022588729858,
                                  -0.2687852084636688,  -0.28991109132766724, -0.10992398858070374,
                                  0.5791637301445007,   0.20270979404449463};
  float angular_velocity_y[11] = {-0.22786562144756317, -0.1880311667919159,  -0.15138500928878784,
                                  -0.11366301774978638, -0.06818123161792755, 5.7508761528879404e-05,
                                  0.12564195692539215,  0.3862893581390381,   0.9425992965698242,
                                  1.9430519342422485,   2.1432816982269287};
  float angular_velocity_z[11] = {0.14619050920009613, 0.15647569298744202,  0.1400030255317688,  0.11495900899171829,
                                  0.0966373011469841,  0.09889957308769226,  0.14427395164966583, 0.26833808422088623,
                                  0.42734161019325256, 0.022022604942321777, -1.5626710653305054};
  float current_yaw[11] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float kf_correction = 0.0;
  float angle_corrections[2] = {0.0, 0.0};
  bool enable_motors = true;
  bool use_external_yaw = true;
  float kR[3] = {1.5, 1.5, 1.0};
  float kOm[3] = {0.12999999523162842, 0.12999999523162842, 0.10000000149011612};
};

/*
 * @brief Struct to store reference data for Test6
 */
struct Test6Data
{
  float force_x[11] = {-0.5239999890327454, -2.006556987762451,  -3.344144105911255, -4.405928134918213,
                       -5.089616298675537,  -5.324000358581543,  -5.089015960693359, -4.407608509063721,
                       -3.346424102783203,  -2.0092592239379883, -0.5220448970794678};
  float force_y[11] = {-7.570000171661377,  -7.217967987060547, -6.196263790130615, -4.6048479080200195,
                       -2.5924386978149414, -0.368533730506897, 1.855224847793579,  3.8613924980163574,
                       5.453768253326416,   6.476649284362793,  6.830000400543213};
  float force_z[11] = {18.60500144958496,  17.87158966064453,   15.743061065673828, 12.427579879760742,
                       8.23508071899414,   3.601945400238037,   -1.030900001525879, -5.2103800773620605,
                       -8.527839660644531, -10.658828735351562, -11.394998550415039};
  float orientation_x[11] = {0.1919247806072235,   0.17779363691806793,  0.14145730435848236, 0.07911645621061325,
                             -0.03236406296491623, -0.29856163263320923, 0.7377450466156006,  0.827484667301178,
                             0.8909380435943604,   0.9413164854049683,   0.9635857343673706};
  float orientation_y[11] = {-0.013816281221807003, -0.08524071425199509, -0.15349476039409637, -0.22097980976104736,
                             -0.2968239486217499,   -0.3621373176574707,  -0.22184395790100098, -0.3840705156326294,
                             -0.3258344829082489,   -0.18916954100131989, -0.005871208384633064};
  float orientation_z[11] = {0.0027021942660212517, 0.1715720146894455,   0.31809231638908386, 0.43473416566848755,
                             0.5227949619293213,    0.6074190139770508,   -0.6365343332290649, -0.39307504892349243,
                             -0.2479761391878128,   -0.13380326330661774, -0.02199784480035305};
  float orientation_w[11] = {0.98130863904953,     0.9652391076087952,  0.924795389175415,    0.869433581829071,
                             0.7984569668769836,   0.6409053206443787,  -0.03662708401679993, -0.11511273682117462,
                             -0.19639040529727936, -0.2454279363155365, -0.2664286196231842};
  float angular_velocity_x[11] = {0.0020870447624474764,  0.001849425956606865,   0.0013905223459005356,
                                  0.0006125122308731079,  -0.0010874252766370773, -0.006499685347080231,
                                  -0.0023437263444066048, -0.0019156448543071747, -0.002867378294467926,
                                  -0.003394216299057007,  -0.003551904112100601};
  float angular_velocity_y[11] = {-0.00031880292226560414, -0.0012092795222997665, -0.0020915213972330093,
                                  -0.0031001705210655928,  -0.004464729689061642,  -0.004650786519050598,
                                  0.009854063391685486,    0.006197452079504728,   0.003493777709081769,
                                  0.0015860107960179448,   -0.00020525031141005456};
  float angular_velocity_z[11] = {0.21595332026481628,  0.20623962581157684,  0.1888391673564911,
                                  0.17844238877296448,  0.19427490234375,     0.346982479095459,
                                  -0.08114977180957794, -0.13559433817863464, -0.16966792941093445,
                                  -0.20964330434799194, -0.23274841904640198};
  float current_yaw[11] = {0.20335820317268372, 0.20335820317268372, 0.20335820317268372, 0.20335820317268372,
                           0.20335820317268372, 0.20335820317268372, 0.20335820317268372, 0.20335820317268372,
                           0.20335820317268372, 0.20335820317268372, 0.20335820317268372};
  float kf_correction = 0.0;
  float angle_corrections[2] = {0.0, 0.0};
  bool enable_motors = true;
  bool use_external_yaw = true;
  float kR[3] = {1.5, 1.5, 1.0};
  float kOm[3] = {0.12999999523162842, 0.12999999523162842, 0.10000000149011612};
};

/*
 * @brief Struct to store reference data for Test7
 */
struct Test7Data
{
  float force_x[11] = {10.947500228881836,  10.259871482849121,  8.387818336486816,  5.5143609046936035,
                       1.9082738161087036,  -2.0551068782806396, -5.998676300048828, -9.536781311035156,
                       -12.323446273803711, -14.086194038391113, -14.65241813659668};
  float force_y[11] = {-3.8005001544952393, -5.772353172302246,  -7.54161262512207,  -8.935220718383789,
                       -9.818855285644531,  -10.100480079650879, -9.756282806396484, -8.819875717163086,
                       -7.382847309112549,  -5.585753440856934,  -3.597893238067627};
  float force_z[11] = {-0.25499963760375977, 1.194225788116455,  2.4968419075012207,  3.525468111038208,
                       4.1809282302856445,   4.394989967346191,  4.149449348449707,   3.4683151245117188,
                       2.4181904792785645,   1.1017704010009766, -0.35691404342651367};
  float orientation_x[11] = {0.11618131399154663, 0.20505118370056152, 0.4320967495441437, 0.5745119452476501,
                             0.46231064200401306, 0.33149445056915283, 0.1803325116634369, -0.11896124482154846,
                             -0.5798372626304626, -0.6647531390190125, 0.7104867696762085};
  float orientation_y[11] = {0.7053380012512207,  0.6383500099182129,   0.4530227482318878,  -0.10385256260633469,
                             -0.3056844472885132, -0.44045379757881165, -0.5451251268386841, -0.5973836183547974,
                             -0.2842378318309784, -0.14779958128929138, 0.08386446535587311};
  float orientation_z[11] = {-0.1189190223813057, -0.14861087501049042, 0.022949347272515297, 0.5424880981445312,
                             0.5831124782562256,  0.5532097816467285,   0.5274716019630432,   0.6418283581733704,
                             0.7608525156974792,  0.7230895161628723,   -0.6933917999267578};
  float orientation_w[11] = {0.6890998482704163,   0.7268961668014526,   0.7794460654258728, 0.6040341854095459,
                             0.5939745903015137,   0.6245564818382263,   0.6261728405952454, 0.46587273478507996,
                             -0.06403983384370804, -0.11575867980718613, 0.0859249085187912};
  float angular_velocity_x[11] = {0.5322777032852173,  0.5868829488754272,  0.3535895347595215, -0.9128600358963013,
                                  -1.1392476558685303, -1.1454377174377441, -1.001406192779541, -0.6465718746185303,
                                  0.39991194009780884, 0.44766315817832947, 0.41450363397598267};
  float angular_velocity_y[11] = {-0.31529873609542847, -0.3596312403678894,  -0.7495870590209961, -0.43825745582580566,
                                  -0.1936522126197815,  -0.08636009693145752, 0.04827471077442169, 0.5123074054718018,
                                  0.5563209056854248,   0.3969888687133789,   0.3799377381801605};
  float angular_velocity_z[11] = {0.09710381925106049, 0.4948006868362427,  3.6318814754486084, 1.490554690361023,
                                  0.33773714303970337, 0.18682341277599335, 0.3988396227359772, 3.365112066268921,
                                  1.6702911853790283,  0.388555109500885,   0.08056710660457611};
  float current_yaw[11] = {-0.030421769246459007, -0.030421769246459007, -0.030421769246459007, -0.030421769246459007,
                           -0.030421769246459007, -0.030421769246459007, -0.030421769246459007, -0.030421769246459007,
                           -0.030421769246459007, -0.030421769246459007, -0.030421769246459007};
  float kf_correction = 1.0;
  float angle_corrections[2] = {0.20000000298023224, 0.30000001192092896};
  bool enable_motors = true;
  bool use_external_yaw = true;
  float kR[3] = {1.5, 1.5, 1.0};
  float kOm[3] = {0.12999999523162842, 0.12999999523162842, 0.10000000149011612};
};
