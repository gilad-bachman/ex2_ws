/*
 * Exercise 2 - ID: 211397054
 *
 * The behavior tree engine: the status every node returns, the node
 * interface, and the control nodes. Generic: it knows nothing about lights,
 * batteries or the robot.
 */
#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace argos {

   /* What a node answers when it is ticked */
   enum class EStatus { SUCCESS, FAILURE, RUNNING };

   /* Every node in the tree: control nodes, leaves and conditions */
   class CNode {

   public:

      virtual ~CNode() = default;

      /* Run one step of this node, and say how it is doing */
      virtual EStatus Tick() = 0;

      /* Stop this node if it is running */
      virtual void Halt() = 0;
   };

   /* A node with children, ticked left to right */
   class CControlNode : public CNode {

   public:

      /* Add a child at the end; returns this node, so children can be chained */
      CControlNode& Add(std::unique_ptr<CNode> pc_child) {
         m_vecChildren.push_back(std::move(pc_child));
         return *this;
      }

   protected:

      std::vector<std::unique_ptr<CNode>> m_vecChildren;
   };

   /*
    * Fallback: try the children in order until one succeeds.
    * SUCCESS as soon as a child succeeds, FAILURE if they all fail.
    * Remembers which child is running (m_unCurrent), and continues it next tick.
    */
   class CFallback : public CControlNode {

   public:

      EStatus Tick() override {
         while (m_unCurrent < m_vecChildren.size()) {
            EStatus eStatus = m_vecChildren[m_unCurrent]->Tick();
            if (eStatus == EStatus::RUNNING) return eStatus;
            if (eStatus == EStatus::SUCCESS) { m_unCurrent = 0; return eStatus; }
            ++m_unCurrent;   // this one failed: try the next
         }
         m_unCurrent = 0;
         return EStatus::FAILURE;   // all of them failed
      }

      void Halt() override {
         if (m_unCurrent < m_vecChildren.size()) m_vecChildren[m_unCurrent]->Halt();
         m_unCurrent = 0;
      }

   private:

      size_t m_unCurrent = 0;
   };

   /*
    * Sequence: run the children in order until one fails.
    * FAILURE as soon as a child fails, SUCCESS if they all succeed.
    * Remembers which child is running (m_unCurrent), and continues it next tick.
    */
   class CSequence : public CControlNode {

   public:

      EStatus Tick() override {
         while (m_unCurrent < m_vecChildren.size()) {
            EStatus eStatus = m_vecChildren[m_unCurrent]->Tick();
            if (eStatus == EStatus::RUNNING) return eStatus;
            if (eStatus == EStatus::FAILURE) { m_unCurrent = 0; return eStatus; }
            ++m_unCurrent;   // this one succeeded: the next
         }
         m_unCurrent = 0;
         return EStatus::SUCCESS;   // all of them succeeded
      }

      void Halt() override {
         if (m_unCurrent < m_vecChildren.size()) m_vecChildren[m_unCurrent]->Halt();
         m_unCurrent = 0;
      }

   private:

      size_t m_unCurrent = 0;
   };

   /*
    * ReactiveFallback: like Fallback, but starts from the first child on every
    * tick. If an earlier child stops failing (it takes over), the later child
    * that was running is halted. Used for interrupts: child 0 is the
    * interrupt (for example a low battery), child 1 the mission.
    */
   class CReactiveFallback : public CControlNode {

   public:

      EStatus Tick() override {
         for (size_t i = 0; i < m_vecChildren.size(); ++i) {
            EStatus eStatus = m_vecChildren[i]->Tick();
            if (eStatus == EStatus::FAILURE) continue;   // no: ask the next child
            if (m_nRunning > static_cast<long>(i))
               m_vecChildren[m_nRunning]->Halt();         // an earlier child takes over
            m_nRunning = (eStatus == EStatus::RUNNING) ? static_cast<long>(i) : -1;
            return eStatus;
         }
         m_nRunning = -1;
         return EStatus::FAILURE;   // all of them failed
      }

      void Halt() override {
         if (m_nRunning >= 0) m_vecChildren[m_nRunning]->Halt();
         m_nRunning = -1;
      }

   private:

      long m_nRunning = -1;   // the child that was RUNNING last tick; -1: none
   };

   /* A node with exactly one child, that changes what the child answers */
   class CDecorator : public CNode {

   public:

      explicit CDecorator(std::unique_ptr<CNode> pc_child) : m_pcChild(std::move(pc_child)) {}

      void Halt() override { m_pcChild->Halt(); }

   protected:

      std::unique_ptr<CNode> m_pcChild;
   };

   /* ForceFailure: FAILURE whenever the child finishes, whatever it answered */
   class CForceFailure : public CDecorator {

   public:

      using CDecorator::CDecorator;

      EStatus Tick() override {
         EStatus eStatus = m_pcChild->Tick();
         return (eStatus == EStatus::RUNNING) ? eStatus : EStatus::FAILURE;
      }
   };

   /*
    * RetryUntilSuccessful: when the child fails, run it again (from the start,
    * on the next tick), until it succeeds. Never fails.
    */
   class CRetryUntilSuccessful : public CDecorator {

   public:

      using CDecorator::CDecorator;

      EStatus Tick() override {
         EStatus eStatus = m_pcChild->Tick();
         return (eStatus == EStatus::FAILURE) ? EStatus::RUNNING : eStatus;
      }
   };

}

#endif
