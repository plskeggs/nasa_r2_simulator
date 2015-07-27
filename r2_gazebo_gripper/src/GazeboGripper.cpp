#include "r2_gazebo_gripper/GazeboGripper.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

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

    prevUpdateTime = _model->GetWorld()->GetSimTime();
    prevContactUpdateTime = prevUpdateTime;

    if (!_sdf->HasAttribute("name"))
    {
        ROS_WARN("GazeboGripper: Plugin name missing");
        pluginName = "GazeboGripperPlugin";
    }
    else
        pluginName = _sdf->GetAttribute("name")->GetAsString().c_str();
    ROS_INFO("Loading GazeboGripper plugin '%s'", pluginName.c_str());

    // get params
    if (!_sdf->HasElement("updateRate"))
    {
        ROS_INFO("%s: plugin missing <updateRate>, default to 1000", pluginName.c_str());
        updateTime = 1 / 1000;
    }
    else
    {
        updateTime = 1 / (_sdf->GetElement("updateRate")->Get<double>());
    }

    if (!_sdf->HasElement("attachWait"))
    {
        ROS_FATAL("%s: plugin missing <attachWait>, cannot proceed", pluginName.c_str());
        return;
    }
    else
    {
        attachWait = _sdf->GetElement("attachWait")->Get<double>();
    }

    if (!_sdf->HasElement("detachWait"))
    {
        ROS_FATAL("%s: plugin missing <detachWait>, cannot proceed", pluginName.c_str());
        return;
    }
    else
    {
        detachWait = _sdf->GetElement("detachWait")->Get<double>();
    }

    if (!_sdf->HasElement("maxRelativeMotionRate"))
    {
        ROS_FATAL("%s: plugin missing <maxRelativeMotionRate>, cannot proceed", pluginName.c_str());
        return;
    }
    else
    {
        maxRelativeMotionRate = _sdf->GetElement("maxRelativeMotionRate")->Get<double>();
    }

    if (!_sdf->HasElement("gripperAttachLink"))
    {
        ROS_FATAL("%s: plugin missing <gripperAttachLink>, cannot proceed", pluginName.c_str());
        return;
    }
    else
    {
        gripperAttachLink = _sdf->GetElement("gripperAttachLink")->Get<std::string>();

        if (!modelPtr->GetLink(gripperAttachLink))
        {
            ROS_FATAL((std::string("%s: plugin couldn't find gripperAttachLink (%s) in model, cannot proceed.\n")
                       + "\tLinks connected with static joints may be combined into one by the URDF parser.").c_str(),
                      pluginName.c_str(), gripperAttachLink.c_str());
            return;
        }

        ROS_INFO("%s: gripperAttachLink: %s", pluginName.c_str(), gripperAttachLink.c_str());
    }

    if (!_sdf->HasElement("gripperLink"))
    {
        ROS_FATAL("%s: plugin missing <gripperLink>, cannot proceed", pluginName.c_str());
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
            ROS_DEBUG("%s: added gripperLink: %s", linkElem->Get<std::string>().c_str(), pluginName.c_str());
        }
        else
        {
            ROS_FATAL((std::string("%s: plugin couldn't find gripperLink (%s) in model, cannot proceed.\n")
                       + "\tLinks connected with static joints may be combined into one by the URDF parser.").c_str(),
                      pluginName.c_str(), linkElem->Get<std::string>().c_str());
            return;
        }

        linkElem = linkElem->GetNextElement("gripperLink");
    }

    if (links.size() < 2)
    {
        ROS_FATAL("%s: plugin requires at least two gripperLinks, cannot proceed", pluginName.c_str());
        return;
    }

    fixedJointPtr = modelPtr->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute");
    fixedJointPtr->SetName(pluginName + "_joint");

    collisionNames.clear();

    // iterate through links we care about contacts for and setup a filter in the contact manager
    for (std::vector<physics::LinkPtr>::iterator linkIt = links.begin(); linkIt != links.end(); ++linkIt)
    {
        for (unsigned int j = 0; j < (*linkIt)->GetChildCount(); ++j)
        {
            physics::CollisionPtr collision = (*linkIt)->GetCollision(j);
            collisionNames.push_back(collision->GetScopedName());
            ROS_DEBUG("%s: Added collision '%s'", pluginName.c_str(), collision->GetScopedName().c_str());
        }
    }

    if (collisionNames.size())
    {
        // The contact manager provides an interface between the physics engine and the links in contact
        physics::ContactManager *mgr = modelPtr->GetWorld()->GetPhysicsEngine()->GetContactManager();

        // Filter only the contacts we care about
        mgr->CreateFilter(pluginName, collisionNames);
        std::sort(collisionNames.begin(), collisionNames.end());
    }

    // Connect a callback for when the world is updated (very regularly)
    worldUpdateEvent = event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboGripper::onUpdate, this));

    ROS_INFO("%s: plugin loaded", pluginName.c_str());
}

void GazeboGripper::Init()
{
    ROS_INFO("Gazebo Gripper plugin initialized");
    ModelPlugin::Init();
}

void GazeboGripper::onUpdate()
{
    physics::ContactManager *mgr = modelPtr->GetWorld()->GetPhysicsEngine()->GetContactManager();

    common::Time currTime = modelPtr->GetWorld()->GetSimTime();
    if ((currTime - prevUpdateTime).Double() >= updateTime)  // limit the rate at which we check contacts
    {
        prevUpdateTime = currTime;

        const std::vector<physics::Contact*>& contacts = mgr->GetContacts();

        std::vector<physics::Contact*> myContacts;
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            // The names of the links in contact
            std::string name1 = contacts[i]->collision1->GetScopedName();
            std::string name2 = contacts[i]->collision2->GetScopedName();

            if (std::find(collisionNames.begin(), collisionNames.end(), name1) != collisionNames.end() ||
                std::find(collisionNames.begin(), collisionNames.end(), name2) != collisionNames.end())
                myContacts.push_back(contacts[i]);
        }

        handleContacts(myContacts);
        mgr->Clear();  // clear the contact list.  This does not happen automatically
    }
}

void GazeboGripper::handleContacts(const std::vector<physics::Contact*>& contacts)
{
    common::Time currTime = modelPtr->GetWorld()->GetSimTime();
    double elapsedTime = (currTime - prevContactUpdateTime).Double();
    prevContactUpdateTime = currTime;

    // Due to jitter in the zero-G simulation, we relax the contact requirement to just one of the gripper links
    int minContactCount = 1;
    if (contacts.size() >= minContactCount)
    {
        ROS_DEBUG("%s: Contacts: %d, duration: %f, attached? %s", pluginName.c_str(), (int)contacts.size(), contactDuration+elapsedTime, attached ? "YES":"NO");

        nonContactDuration = 0;
        contactDuration += elapsedTime;

        // We were not attached previously, but now there are contacts.  Check to make sure there
        // is no relative movement for attachWait seconds.
        if (!attached)
        {
            // Constructing a list of world objects in contact and the number of contacts on each object
            std::map<std::string, physics::Collision*> cc;
            std::map<std::string, int> contactCounts;

            // identify gripper links colliding with the world parts
            for (unsigned int i = 0; i < contacts.size(); ++i)
            {
                std::string name1 = contacts[i]->collision1->GetScopedName();
                std::string name2 = contacts[i]->collision2->GetScopedName();

                // If the link is not in collisionNames, it is part of the world
                std::vector<std::string>::iterator citer = std::find(collisionNames.begin(), collisionNames.end(), name1);
                if (citer == collisionNames.end())
                {
                    cc[name1] = contacts[i]->collision1;
                    contactCounts[name1] += 1;
                }

                citer = std::find(collisionNames.begin(), collisionNames.end(), name2);
                if (citer == collisionNames.end())
                {
                    cc[name2] = contacts[i]->collision2;
                    contactCounts[name2] += 1;
                }
            }

            // Iterate through the world links in contact.  If there are at least
            // minContactCount contacts, attach the gripper to the world link
            for(std::map<std::string, int>::iterator iter = contactCounts.begin(); iter != contactCounts.end(); ++iter)
            {
                // Insufficient number of contacts on this link
                if (iter->second < minContactCount)
                    continue;

                // Compute the relative motion rate between the gripper and the world link in contact
                double relMotionRate;

                math::Pose diff = cc[iter->first]->GetLink()->GetWorldPose() - modelPtr->GetLink(gripperAttachLink)->GetWorldPose();
                if (contactDuration <= elapsedTime)  // first contact with the world.  Relative motion is zero
                    relMotionRate = 0.0;
                else
                    relMotionRate = (diff - prevDiff).pos.GetSquaredLength() / elapsedTime;

                prevDiff = diff;

                // Links are moving too fast to attach
                if (relMotionRate > maxRelativeMotionRate)
                {
                    ROS_DEBUG("%s: gripper contact relative motion too great (%f).", pluginName.c_str(), relMotionRate);
                    contactDuration = 0;
                }
                else if (contactDuration >= attachWait) // small relative motion for a sufficient amount of time
                {
                    handleAttach(cc[iter->first]->GetLink());
                    break; // only handle one attachment
                }
            }
        }
    }
    else  // too few contacts
    {
        contactDuration = 0;

        if (attached)
        {
            nonContactDuration += elapsedTime;
            if (nonContactDuration >= detachWait)
                handleDetach();
        }
    }
}

void GazeboGripper::handleAttach(physics::LinkPtr linkPtr)
{
    attached = true;

    fixedJointPtr->Load(modelPtr->GetLink(gripperAttachLink), linkPtr, math::Pose(0, 0, 0, 0, 0, 0));
    fixedJointPtr->Init();
    fixedJointPtr->SetHighStop(0, 0);
    fixedJointPtr->SetLowStop(0, 0);

    ROS_INFO("%s: Gripper attached", pluginName.c_str());
}

void GazeboGripper::handleDetach()
{
    attached = false;
    fixedJointPtr->Detach();
    ROS_INFO("%s: Gripper detached", pluginName.c_str());
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGripper)
