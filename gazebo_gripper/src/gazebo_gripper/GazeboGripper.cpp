#include <gazebo_gripper/GazeboGripper.h>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>

using namespace gazebo;

GazeboGripper::GazeboGripper()
    : ModelPlugin()
    , attached(false)
    , contactDuration(0.)
    , nonContactDuration(0.)
{
}

GazeboGripper::~GazeboGripper()
{
}

void GazeboGripper::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    modelPtr = _model;

    prevUpdateTime = _model->GetWorld()->SimTime();
    prevContactUpdateTime = prevUpdateTime;

    // get params
    if (!_sdf->HasElement("updateRate"))
    {
        ROS_INFO("GazeboGripper plugin missing <updateRate>, default to 1000");
        updateTime = 1/1000;
    }
    else
    {
        updateTime = 1/(_sdf->GetElement("updateRate")->Get<double>());
    }

    if (!_sdf->HasElement("attachWait"))
    {
        ROS_FATAL("GazeboGripper plugin missing <attachWait>, cannot proceed");
        return;
    }
    else
    {
        attachWait = _sdf->GetElement("attachWait")->Get<double>();
    }

    if (!_sdf->HasElement("detachWait"))
    {
        ROS_FATAL("GazeboGripper plugin missing <detachWait>, cannot proceed");
        return;
    }
    else
    {
        detachWait = _sdf->GetElement("detachWait")->Get<double>();
    }

    if (!_sdf->HasElement("maxRelativeMotionRate"))
    {
        ROS_FATAL("GazeboGripper plugin missing <maxRelativeMotionRate>, cannot proceed");
        return;
    }
    else
    {
        maxRelativeMotionRate = _sdf->GetElement("maxRelativeMotionRate")->Get<double>();
    }

    if (!_sdf->HasElement("gripperAttachLink"))
    {
        ROS_FATAL("GazeboGripper plugin missing <gripperAttachLink>, cannot proceed");
        return;
    }
    else
    {
        gripperAttachLink = _sdf->GetElement("gripperAttachLink")->Get<std::string>();
        if (!modelPtr->GetLink(gripperAttachLink))
        {
            ROS_FATAL((std::string("GazeboGripper plugin couldn't find gripperAttachLink (%s) in model, cannot proceed.\n")
                                   + "\tLinks connected with static joints may be combined into one by the URDF parser.").c_str(),
                       gripperAttachLink.c_str());
            return;
        }
        ROS_DEBUG("GazeboGripper plugin gripperAttachLink: %s", gripperAttachLink.c_str());
    }

    if (!_sdf->HasElement("gripperLink"))
    {
        ROS_FATAL("GazeboGripper plugin missing <gripperLink>, cannot proceed");
        return;
    }

    std::vector<physics::LinkPtr> links;
    sdf::ElementPtr linkElem = _sdf->GetElement("gripperLink");
    while (linkElem)
    {
        physics::LinkPtr link;
        if (link = modelPtr->GetLink(linkElem->Get<std::string>()))
        {
            links.push_back(link);
            ROS_DEBUG("GazeboGripper plugin added gripperLink: %s", linkElem->Get<std::string>().c_str());
        }
        else
        {
            ROS_FATAL((std::string("GazeboGripper plugin couldn't find gripperLink (%s) in model, cannot proceed.\n")
                                   + "\tLinks connected with static joints may be combined into one by the URDF parser.").c_str(),
                       linkElem->Get<std::string>().c_str());
            return;
        }

        linkElem = linkElem->GetNextElement("gripperLink");
    }

    if (links.size() < 2)
    {
        ROS_FATAL("GazeboGripper plugin requires at least two gripperLinks, cannot proceed");
        return;
    }

    fixedJointPtr = modelPtr->GetWorld()->Physics()->CreateJoint("revolute");

    // @todo: fix collision
    /*
    physics::CollisionPtr collision;
    std::map<std::string, physics::CollisionPtr>::iterator collIter;

    for (std::vector<physics::LinkPtr>::iterator linkIt = links.begin(); linkIt != links.end(); ++linkIt)
    {
        for (unsigned int j = 0; j < (*linkIt)->GetChildCount(); ++j)
        {
            collision = (*linkIt)->GetCollision(j);
            collIter = collisionPtrs.find(collision->GetScopedName());
            if (collIter != collisionPtrs.end())
            {
                continue;
            }

            //collision->SetContactsEnabled(true);
            collision->Init()
            connectionPtrs.push_back(collision->ConnectContact(boost::bind(&GazeboGripper::onContact, this, _1, _2)));
            collisionPtrs[collision->GetScopedName()] = collision;
            ROS_DEBUG("GazeboGripper plugin added collision: %s", collision->GetScopedName().c_str());
        }
    }

    this->connectionPtrs.push_back(event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboGripper::onUpdate, this)));
   */
    ROS_INFO("Gazebo Gripper plugin loaded");
}

void GazeboGripper::Init()
{
    ROS_INFO("Gazebo Gripper plugin initialized");
}

void GazeboGripper::onUpdate()
{
    common::Time currTime = modelPtr->GetWorld()->SimTime();

    if ((currTime - prevUpdateTime).Double() < updateTime)
    {
        return;
    }

    handleContact();

    prevUpdateTime = currTime;
}

void GazeboGripper::handleContact()
{
    common::Time currTime = modelPtr->GetWorld()->SimTime();
    double elapsedTime = (currTime - prevContactUpdateTime).Double();
    prevContactUpdateTime = currTime;

    if (contacts.size() >= 2)
    {
        nonContactDuration = 0;
        contactDuration += elapsedTime;
        ROS_DEBUG("GazeboGripper plugin contact detected. Contacts: %d, duration: %f", (int)contacts.size(), contactDuration);

        if (!attached)
        {
            // check to make sure there is no relative movement for attachWait time
            std::map<std::string, physics::Collision*> cc;
            std::map<std::string, int> contactCounts;
            std::map<std::string, int>::iterator iter;

            // identify gripper links colliding with each external part
            for (unsigned int i = 0; i < contacts.size(); ++i)
            {
                std::string name1 = contacts[i].collision1->GetScopedName();
                std::string name2 = contacts[i].collision2->GetScopedName();

                if (collisionPtrs.find(name1) == collisionPtrs.end())
                {
                    cc[name1] = contacts[i].collision1;
                    contactCounts[name1] += 1;
                }
                if (collisionPtrs.find(name2) == collisionPtrs.end())
                {
                    cc[name2] = contacts[i].collision2;
                    contactCounts[name2] += 1;
                }
            }

            // verify 2 gripper links touching part before attaching
            iter = contactCounts.begin();
            while (iter != contactCounts.end())
            {
                if (iter->second < 2)
                {
                    contactCounts.erase(iter++);
                }
                else
                {
                    ignition::math::Pose3d diff = cc[iter->first]->GetLink()->WorldPose() - modelPtr->GetLink(gripperAttachLink)->WorldPose();

                    double relMotionRate; // find relative motion rate
                    if (contactDuration <= elapsedTime)
                    {
                        // first time in
                        relMotionRate = 0;
                    }
                    else
                    {
                        relMotionRate = (diff - prevDiff).Pos().SquaredLength() / elapsedTime;
                    }
                    prevDiff = diff;

                    if (relMotionRate > maxRelativeMotionRate)
                    {
                        // moving too fast
                        ROS_DEBUG("GazeboGripper plugin gripper contact relative motion too great (%f).", relMotionRate);
                        contactDuration = 0;
                    }
                    else if (contactDuration >= attachWait)
                    {
                        handleAttach(cc[iter->first]->GetLink());
                        break; // only handle one attachment
                    }

                    ++iter;
                }
            }
        }
    }
    else
    {
        contactDuration = 0;

        if (attached)
        {
            // track time before detach
            nonContactDuration += elapsedTime;

            if (nonContactDuration >= detachWait)
            {
                handleDetach();
            }
        }
    }

    contacts.clear();
}

void GazeboGripper::handleAttach(physics::LinkPtr linkPtr)
{
    attached = true;

    fixedJointPtr->Load(modelPtr->GetLink(gripperAttachLink), linkPtr, ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
    fixedJointPtr->Init();
    fixedJointPtr->SetUpperLimit(0, 0);
    fixedJointPtr->SetLowerLimit(0, 0);

    ROS_INFO("GazeboGripper attached to %s", linkPtr->GetName().c_str());
}

void GazeboGripper::handleDetach()
{
    attached = false;
    fixedJointPtr->Detach();
    ROS_INFO("GazeboGripper detached");
}

void GazeboGripper::onContact(const std::string& _collisionName, const physics::Contact& _contact)
{
    contacts.push_back(_contact);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGripper)
