#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

// LQR controller plugin
// It should be defined inside sdf/urdf robot model
// sends control signal every 20ms

namespace gazebo
{
    class LQRControllerPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
            this->model = _parent;

            physics::LinkPtr pole_link = this->model->GetLink("dummy_link");
            if (!pole_link) {
                gzerr << "No link named 'base' in model " << this->model->GetName() << "\n";
                return;
            }

            if (!this->model->GetJoint("right_wheel_joint") || !this->model->GetJoint("left_wheel_joint")) {
                gzerr << "Joints are not found in the model. Please check their names.\n";
                return;
            }

            // Retrieve LQR gains and joint name from SDF/URDF
            if (_sdf->HasElement("k1")) k1 = _sdf->Get<double>("k1");
            if (_sdf->HasElement("k2")) k2 = _sdf->Get<double>("k2");
            if (_sdf->HasElement("k3")) k3 = _sdf->Get<double>("k3");
            if (_sdf->HasElement("k4")) k4 = _sdf->Get<double>("k4");

            // Connect to update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&LQRControllerPlugin::OnUpdate, this)
            );

            if (!ros::isInitialized()){
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }
            this->nh.reset(new ros::NodeHandle("gazebo_client"));

            // Initialize publisher
            this->pub = nh->advertise<std_msgs::Float64MultiArray>("/state", 1);
        }

        void OnUpdate() {
                // Get state variables
                double x = this->model->WorldPose().Pos().X();
                double x_dot = this->model->WorldLinearVel().X();

                // Assuming you have a link named "pole_link"
                physics::LinkPtr pole_link = this->model->GetLink("dummy_link");

                // Get the pole's pose and velocity
                ignition::math::Pose3d pole_pose = pole_link->WorldPose();
                ignition::math::Vector3d pole_linear_vel = pole_link->WorldLinearVel();
                ignition::math::Vector3d pole_angular_vel = pole_link->WorldAngularVel();

                // Extract the relevant angle and angular velocity
                // NOTE: The exact computation might differ based on your model setup!
                double phi = -pole_pose.Rot().Pitch(); // or Roll() or Yaw() based on your setup
                double phi_dot = -pole_angular_vel.Y(); // or X() or Z() based on your setup


                // Compute control input u using LQR law

                // Apply control input to robot (adapt as per your robot model)
                if(iteration_counter % 20 == 0) { // Apply control input every 20 iterations
                    u = (k1*(x - 5.0) + k2*phi + k3*x_dot + k4*phi_dot)/2.0;
                }

                this->model->GetJoint("right_wheel_joint")->SetForce(0, u);
                this->model->GetJoint("left_wheel_joint")->SetForce(0, u);
                std_msgs::Float64MultiArray msg;
                msg.data.clear();
                msg.data.push_back(iteration_counter);
                msg.data.push_back(x);
                msg.data.push_back(phi);
                msg.data.push_back(x_dot);
                msg.data.push_back(phi_dot);
                msg.data.push_back(u * 2.0);
                if (iteration_counter <= 20000) {
                    pub.publish(msg);
                }



            // Update the iteration counter

                iteration_counter++;
                
        }

    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        // LQR gains
        double k1 = 0.0, k2 = 0.0, k3 = 0.0, k4 = 0.0;
        double u = 0.0;
        int iteration_counter = 0;
        std::unique_ptr<ros::NodeHandle> nh;
        ros::Publisher pub;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(LQRControllerPlugin)
}