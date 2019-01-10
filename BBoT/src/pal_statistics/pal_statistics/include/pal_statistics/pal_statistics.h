/**************************************************************************
**
**  Author: victor
**  Created on: 2018/06/06
**
**  Copyright (c) 2018 PAL Robotics SL. All Rights Reserved
**************************************************************************/

#ifndef _PAL_STATISTICS_H_
#define _PAL_STATISTICS_H_

#include <ros/ros.h>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/thread.hpp>
#include <pal_statistics_msgs/Statistics.h>
#include <pal_statistics/pal_statistics_utils.h>
#include <gtest/gtest_prod.h>

namespace pal_statistics
{
/**
 * @brief The StatisticsRegistry class reads the value of registered variables and
 * publishes them on the specified topic.
 *
 * @warning Functions are not real-time safe unless stated.
 *
 * @warning Registering and enabling more than one variable with the same name is not
 * supported. Multiple variables with the same name can be registered, only if there's
 * only one of them enabled at any time.
 *
 * If you are using repeated names, it's better to use the Id of the registered variables
 * or the RegistratonRAII for unregister/disable
 */
class StatisticsRegistry : public boost::enable_shared_from_this<StatisticsRegistry>
{
public:
  StatisticsRegistry(const std::string &topic);

  virtual ~StatisticsRegistry();
  
  /**
   * @brief registerVariable Specialization for double*, the most common case, to avoid
   * going through a boost function call to read the variable
   */
  IdType registerVariable(const std::string &name, const double * variable, RegistrationsRAII *bookkeeping = NULL,
                          bool enabled = true)
  {
    return registerInternal(name, VariableHolder(variable), bookkeeping, enabled);
  }

  /**
   * @brief registerFunction Adds a function that returns double with the specified name
   * @param bookkeeping same as in registerVariable
   */
  IdType registerFunction(const std::string &name, const boost::function<double()> &funct,
                        RegistrationsRAII *bookkeeping = NULL, bool enabled = true);


  void unregisterVariable(const std::string &name, RegistrationsRAII *bookkeeping = NULL);
  void unregisterVariable(IdType id, RegistrationsRAII *bookkeeping = NULL);

  /**
   * @brief publish Reads the values of all registered variables and publishes them to the
   * topic associated to this object.
   * @warning This function may lock if another thread is adding/removing registered
   * variables, use publishAsync if you need RT safety
   */
  void publish();

  /**
   * @brief publishAsync Capture data and flag it to be published by the publisher thread.
   * Real-Time safe.
   * @warning If the mutex cannot be acquired, data will not be captured. Should
   * only happen if other threads are registering/unregistering or publishing variables.
   * @return true if mutex and data could be acquired, false otherwise
   */
  bool publishAsync();

  /**
   * @brief startPublishThread creates and starts the publisherThread. The user still has
   * to call publishAsync each time a message must be publisher.
   */
  void startPublishThread();

  /**
   * @brief createMsg creates a Statistics message from the registered variables, useful
   * for debugging
   * @return
   */
  pal_statistics_msgs::Statistics createMsg();
  
  /**
   * @brief publishCustomStatistic publishes a one-time statistic
   */
  template <typename T>
  void publishCustomStatistic(const std::string &name, T value)
  {
    pal_statistics_msgs::Statistics msg;
    pal_statistics_msgs::Statistic stat;
    stat.name = name;
    stat.value = static_cast<double>(value);
    msg.statistics.push_back(stat);
    msg.header.stamp = ros::Time::now();
    pub_.publish(msg);
  }
  
  /**
   * @brief publishCustomStatistic publishes a one-time statistics msg
   */
  void publishCustomStatistics(const pal_statistics_msgs::Statistics &msg)
  {
    pub_.publish(msg);    
  }
  
  /**
   * These functions disable/enable the publication of one or more variables
   * 
   * They are RT safe and thread safe.
   * They guarantee that on the next publish (or publishAsync) call, the specified variables will 
   * or will not be read and published.
   * 
   * @warning If a publish is being executed while this function is being run, 
   * it will not take into account these modifications.
   * 
   * If you need a deterministic way of preventing a variable from being published, 
   * you need to unregister it, but it is not RT safe.
   */
  bool enable(const IdType &id);  
  bool disable(const IdType &id);

private:
  /**
   * @brief updateMsgUnsafe Updates the internal message variable without acquiring the
   * mutex Should only be used if the mutex has already been acquired by the thread
   * calling this
   */
  void updateMsgUnsafe();
  
  void updateMsg(pal_statistics_msgs::Statistics &msg, bool smart_fill = false);

  void publisherThreadCycle();
  
  void startPublishThreadImpl();

  IdType registerInternal(const std::string &name, VariableHolder &&variable, RegistrationsRAII *bookkeeping, bool enabled);
  
  bool setEnabledmpl(const IdType &id, bool enabled);
  
  /**
   * @brief handlePendingDisables Empties by handling the queue of disabled/enabled ids.
   */
  void handlePendingDisables(const boost::unique_lock<boost::mutex> &data_lock);
  
  ros::NodeHandle nh_;

  boost::mutex data_mutex_;
  RegistrationList registration_list_;
    
  struct EnabledId
  {
    //Can't use a pair because it's not trivially copiable
    IdType id;
    bool enabled;
  };
  
  /**
   * @brief disabled_ids_ this is used to keep track of enabled/disabled variables in a
   * lock free way
   *
   * enable/disable need to write, but they cannot be locked, and cannot be
   * skipped if they fail to acquire a mutex. 
   * Therefore they write to a lockfree  structure. 
   * This structure is processed in the next publish or publishAsync that has
   * the write lock and can modify shared structures.
   */
  LockFreeQueue<EnabledId> enabled_ids_;
  
  
  // To avoid deadlocks, should always be acquired after data_mutex_
  boost::mutex pub_mutex_;
  ros::Publisher pub_;
  bool is_data_ready_;
  boost::shared_ptr<boost::thread> publisher_thread_;
  pal_statistics_msgs::Statistics msg_;
  
  // Internal stats
  unsigned int publish_async_attempts_;
  unsigned int publish_async_failures_;
  double last_async_pub_duration_;
  RegistrationsRAII internal_stats_raii_;

  FRIEND_TEST(PalStatisticsTest, stressAsync);
};
}  // namespace pal_statistics
#endif
