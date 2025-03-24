#include "kinosplines/kdtp_init.h"

kdtp::LocalPathPtr compute_local_path(std::vector<double> from_pos, std::vector<double> from_vel, std::vector<double> from_acc,
                          std::vector<double> to_pos, std::vector<double> to_vel, std::vector<double> to_acc, std::vector<double> max_values){

    srand(time(NULL));
    kdtp::RobotPtr robot(new kdtp::Robot("MDT_ROBOT"));

    // Number of degrees of freedom of the whole system
    size_t NbDof= from_vel.size();

    for(size_t i=0; i<NbDof; i++)
    {
        Eigen::VectorXd const_dof(6);
        double c_min, c_max;

        if(from_pos[i] < to_pos[i])
        {
            c_min = from_pos[i] ;
            c_max = to_pos[i];
        }
        else
        {
            c_min = to_pos[i] ;
            c_max = from_pos[i] ;
        }
        // Constraints to respect: Pos(min, max), Vmax, Amax, Jmax, Smax
        //const_dof<< c_min, c_max, 10.0, 3.0, 15, 30;
        const_dof<< c_min, c_max, max_values[i*5], max_values[i*5+1], max_values[i*5+2], max_values[i*5+3];

        //Adding the new degree of freedom
        kdtp::DofPtr dof(new kdtp::Dof(const_dof(0),const_dof(1),const_dof(2),const_dof(3),const_dof(4),const_dof(5), (max_values[i*5+4]==1.0)));
        robot->addDof(dof);
    }

    //Creating States
    kdtp::StatePtr init(new kdtp::State(robot));
    kdtp::StatePtr end(new kdtp::State(robot));
    init->incremental_shoot();
    end->incremental_shoot();

    //Initializing values of the kinospline
    for(size_t i =0; i< NbDof; i++)
    {
        // Initializing position
        init->setPosition(i, from_pos[i]);
        end->setPosition( i, to_pos[i]);
        //Initializing Velocity
        init->setVelocity(i, from_vel[i]);
        end->setVelocity(i, to_vel[i]);
        //Initializing Acceleration
        init->setAcceleration(i, from_acc[i]);
        end->setAcceleration(i, to_acc[i]);
    }

    kdtp::LocalPathPtr LP(new kdtp::LocalPath(robot,init,end));

    return LP;
}