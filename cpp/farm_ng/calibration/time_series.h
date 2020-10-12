#ifndef FARM_NG_CALIBRATION_TIME_SERIES_H_
#define FARM_NG_CALIBRATION_TIME_SERIES_H_
#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/util/time_util.h>
#include <deque>

namespace farm_ng {
template <typename ValueT>
class TimeSeries {
 public:
  typedef std::deque<ValueT> ContainerT;
  typedef typename ContainerT::const_iterator const_iterator;
  static bool Compare(const ValueT& lhs, const ValueT& rhs) {
    return lhs.stamp() < rhs.stamp();
  }

  const_iterator lower_bound(const google::protobuf::Timestamp& stamp) const {
    ValueT x;
    x.mutable_stamp()->CopyFrom(stamp);
    return std::lower_bound(series_.begin(), series_.end(), x,
                            [](const ValueT& lhs, const ValueT& rhs) {
                              return lhs.stamp() < rhs.stamp();
                            });
  }

  const_iterator upper_bound(const google::protobuf::Timestamp& stamp) const {
    return std::upper_bound(
        series_.begin(), series_.end(), stamp,
        [](const google::protobuf::Timestamp& stamp, const ValueT& rhs) {
          return stamp < rhs.stamp();
        });
  }

  std::pair<const_iterator, const_iterator> find_range(
      google::protobuf::Timestamp begin_stamp,
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

 private:
  ContainerT series_;
};
}  // namespace farm_ng
#endif
