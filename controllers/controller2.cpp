/*
 * Exercise 2 - ID: 211397054
 *
 * The starting point: every sensor and actuator is set up, the referee keeps
 * the battery up to date, and the robot stands still. Replace ControlStep() with
 * your executive: tick your behavior tree once per step.
 */
#include "controller2.hpp"

namespace argos {

   namespace {

      /* A rangefinder's label as an index 0..7, whichever type the label is */
      [[maybe_unused]] size_t LabelIndex(UInt8 un_label) { return un_label; }
      [[maybe_unused]] size_t LabelIndex(const std::string& str_label) { return std::stoul(str_label); }
   }

   void Controller2::Init(TConfigurationNode& t_tree) {
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera->Enable();
      m_vecOrder = ReadLightOrder(t_tree);
      m_sBlackboard.Order = m_vecOrder;

      m_pcActuators = std::make_unique<CActuators>(m_pcWheels, m_pcColoredLEDs);
      m_pcRNG = CRandom::CreateRNG("argos");

      /*
       * The tree. Each light is the same subtree (mission.hpp), given its
       * place in the order and which light it is.
       *
       *    ReactiveFallback
       *       Recharge                 battery low: go home, charge (halts the mission)
       *       Sequence                 the mission
       *          VisitLight(0)
       *          VisitLight(1)
       *          VisitLight(2)
       *          ComeHome              go home, stand still 2.5 s
       */
      SMissionContext sCtx{ m_sBlackboard, *m_pcActuators, m_pcRNG };
      auto pcMission = std::make_unique<CSequence>();
      for (size_t i = 0; i < m_vecOrder.size(); ++i)
         pcMission->Add(MakeVisitLight(sCtx, i, m_vecOrder[i]));
      pcMission->Add(MakeComeHome(sCtx));

      auto pcRoot = std::make_unique<CReactiveFallback>();
      pcRoot->Add(MakeRecharge(sCtx));
      pcRoot->Add(std::move(pcMission));
      m_pcRoot = std::move(pcRoot);
   }

   void Controller2::ControlStep() {
      UpdateBlackboard();
      if (m_pcRoot) m_pcRoot->Tick();
   }

   void Controller2::UpdateBlackboard() {
      /* Positioning: where the robot is, and which way it faces */
      const CCI_PositioningSensor::SReading& sPos = m_pcPositioning->GetReading();
      m_sBlackboard.Position.Set(sPos.Position.GetX(), sPos.Position.GetY());
      CRadians cZ, cY, cX;
      sPos.Orientation.ToEulerAngles(cZ, cY, cX);
      m_sBlackboard.Heading = cZ;

      /* Rangefinders: nothing seen reads as the full range */
      m_sBlackboard.Proximity.fill(0.2);
      m_pcRangefinders->Visit([this](const CCI_PiPuckRangefindersSensor::SInterface& s) {
         size_t unIndex = LabelIndex(s.Label);
         if (unIndex < m_sBlackboard.Proximity.size())
            m_sBlackboard.Proximity[unIndex] = s.Proximity;
      });

      /* Camera: copy the blobs it sees this step */
      m_sBlackboard.Blobs.clear();
      for (const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* pcBlob : m_pcCamera->GetReadings().BlobList)
         m_sBlackboard.Blobs.push_back(*pcBlob);

      /* Sightings: every light the camera sees, whichever leaf runs, goes on the
         blackboard in the world frame (the camera's distance is in cm) */
      for (const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob& sBlob : m_sBlackboard.Blobs) {
         for (size_t i = 0; i < m_sBlackboard.Order.size(); ++i) {
            if (!(sBlob.Color == m_sBlackboard.Order[i].Color)) continue;
            CVector2 cOffset(sBlob.Distance / 100.0, m_sBlackboard.Heading + sBlob.Angle);
            m_sBlackboard.LightPosition[i] = m_sBlackboard.Position + cOffset;
            m_sBlackboard.LightKnown[i] = true;
         }
      }

      /* Battery, and where the charger is */
      m_sBlackboard.BatteryLevel = Battery().Level();
      m_sBlackboard.Home = Battery().Home();

      /* Clock */
      m_sBlackboard.Time = m_pcSystem->GetTime();
   }

   REGISTER_CONTROLLER(Controller2, "controller2");

}
