#ifndef FARM_NG_CALIBRATION_TIME_SERIES_H_
#define FARM_NG_CALIBRATION_TIME_SERIES_H_
#include <algorithm>
#include <deque>

#include <glog/logging.h>
#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/util/time_util.h>

namespace farm_ng {
template <typename ValueT>
class TimeSeries {
 public:
  typedef std::deque<ValueT> ContainerT;
  typedef typename ContainerT::const_iterator const_iterator;
  typedef std::pair<const_iterator, const_iterator> RangeT;
  static bool Compare(const ValueT& lhs, const ValueT& rhs) {
    return lhs.stamp() < rhs.stamp();
  }

  bool empty() const { return series_.empty(); }
  const_iterator lower_bound(const google::protobuf::Timestamp& stamp) const {
    ValueT x;
    x.mutable_stamp()->CopyFrom(stamp);
    return std::lower_bound(series_.begin(), series_.end(), x,
                            &TimeSeries::Compare);
  }

  const_iterator upper_bound(const google::protobuf::Timestamp& stamp) const {
    return std::upper_bound(
        series_.begin(), series_.end(), stamp,
        [](const google::protobuf::Timestamp& stamp, const ValueT& rhs) {
          return stamp < rhs.stamp();
        });
  }

  RangeT find_range(google::protobuf::Timestamp begin_stamp,
                    google::protobuf::Timestamp end_stamp) const {
    auto begin_it = lower_bound(begin_stamp);
    VLOG(2) << "begin delta milliseconds: "
            << google::protobuf::util::TimeUtil::DurationToMilliseconds(
                   begin_it->stamp() - begin_stamp);
    auto end_it = upper_bound(end_stamp);
    if (end_it != series_.end()) {
      VLOG(2) << "end delta milliseconds: "
              << google::protobuf::util::TimeUtil::DurationToMilliseconds(
                     end_it->stamp() - end_stamp);
    }

    return std::make_pair(begin_it, end_it);
  }

  void insert(const ValueT& value) {
    series_.insert(upper_bound(value.stamp()), value);
  }

  const_iterator begin() const { return series_.begin(); }

  const_iterator end() const { return series_.end(); }

  size_t size() const { return series_.size(); }

  void RemoveBefore(google::protobuf::Timestamp begin_stamp) {
    if (series_.empty()) {
      return;
    }
    if (begin()->stamp() > begin_stamp) {
      return;
    }
    series_.erase(begin(), lower_bound(begin_stamp));
  }

  std::optional<ValueT> FindNearest(
      google::protobuf::Timestamp stamp,
      google::protobuf::Duration time_window) const {
    auto begin_range = stamp - time_window;

    auto end_range = stamp + time_window;

    auto range = find_range(begin_range, end_range);

    auto closest = range.first;
    double score = 1e10;
    while (range.first != range.second) {
      double score_i = google::protobuf::util::TimeUtil::DurationToMilliseconds(
          range.first->stamp() - stamp);
      if (score_i < score) {
        score = score_i;
        closest = range.first;
      }
      range.first++;
    }
    if (closest != end()) {
      return *closest;
    }
    return std::optional<ValueT>();
  }

 private:
  ContainerT series_;
};  // namespace farm_ng
}  // namespace farm_ng
#endif
