#ifndef FARM_NG_BLOBSTORE_H_
#define FARM_NG_BLOBSTORE_H_

#include <map>
#include <stdexcept>

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include <boost/filesystem.hpp>
#include "glog/logging.h"

#include "farm_ng_proto/tractor/v1/resource.pb.h"

namespace farm_ng {

using farm_ng_proto::tractor::v1::Resource;

// TODO use proto enum here.
enum class BucketId {
  kLogs,
  kCalibrationDatasets,
  kApriltagRigModels,
  kBaseToCameraModels,
  kVideoDatasets
};

boost::filesystem::path GetBlobstoreRoot();
boost::filesystem::path GetBucketRelativePath(BucketId id);
boost::filesystem::path GetBucketAbsolutePath(BucketId id);

void WriteProtobufToJsonFile(const boost::filesystem::path& path,
                             const google::protobuf::Message& proto);

void WriteProtobufToBinaryFile(const boost::filesystem::path& path,
                               const google::protobuf::Message& proto);

template <typename ProtobufT>
const std::string& ContentTypeProtobufBinary() {
  // cache the content_type string on first call.
  static std::string content_type =
      std::string("application/protobuf; type=type.googleapis.com/") +
      ProtobufT::descriptor()->full_name();
  return content_type;
}

template <typename ProtobufT>
const std::string& ContentTypeProtobufJson() {
  // cache the content_type string on first call.
  static std::string content_type =
      std::string("application/json; type=type.googleapis.com/") +
      ProtobufT::descriptor()->full_name();
  return content_type;
}

// Returns a bucket-relative path guaranteed to be unique, with a parent
// directory created if necessary.
boost::filesystem::path MakePathUnique(boost::filesystem::path root,
                                       boost::filesystem::path path);
template <typename ProtobufT>
farm_ng_proto::tractor::v1::Resource WriteProtobufAsJsonResource(
    BucketId id, const std::string& path, const ProtobufT& message) {
  farm_ng_proto::tractor::v1::Resource resource;
  resource.set_content_type(ContentTypeProtobufJson<ProtobufT>());

  boost::filesystem::path write_path =
      MakePathUnique(GetBucketAbsolutePath(id), path);
  write_path += ".json";
  resource.set_path((GetBucketRelativePath(id) / write_path).string());
  WriteProtobufToJsonFile(GetBucketAbsolutePath(id) / write_path, message);
  return resource;
}

template <typename ProtobufT>
farm_ng_proto::tractor::v1::Resource WriteProtobufAsBinaryResource(
    BucketId id, const std::string& path, const ProtobufT& message) {
  farm_ng_proto::tractor::v1::Resource resource;
  resource.set_content_type(ContentTypeProtobufBinary<ProtobufT>());

  boost::filesystem::path write_path =
      MakePathUnique(GetBucketAbsolutePath(id), path);
  write_path += ".pb";
  resource.set_path((GetBucketRelativePath(id) / write_path).string());
  WriteProtobufToBinaryFile(GetBucketAbsolutePath(id) / write_path, message);
  return resource;
}

template <typename ProtobufT>
ProtobufT ReadProtobufFromJsonFile(const boost::filesystem::path& path) {
  LOG(INFO) << "Loading (json proto): " << path.string();
  std::ifstream json_in(path.string());
  CHECK(json_in) << "Could not open path: " << path.string();
  std::string json_str((std::istreambuf_iterator<char>(json_in)),
                       std::istreambuf_iterator<char>());

  CHECK(!json_str.empty()) << "Did not load any text from: " << path.string();
  google::protobuf::util::JsonParseOptions options;

  ProtobufT message;
  auto status =
      google::protobuf::util::JsonStringToMessage(json_str, &message, options);
  CHECK(status.ok()) << status << " " << path.string();

  return message;
}

template <typename ProtobufT>
ProtobufT ReadProtobufFromBinaryFile(const boost::filesystem::path& path) {
  LOG(INFO) << "Loading (binary proto) : " << path.string();
  std::ifstream bin_in(path.string(), std::ifstream::binary);
  CHECK(bin_in) << "Could not open path: " << path.string();
  std::string bin_str((std::istreambuf_iterator<char>(bin_in)),
                      std::istreambuf_iterator<char>());

  CHECK(!bin_str.empty()) << "Did not load any text from: " << path.string();
  ProtobufT message;
  CHECK(message.ParseFromString(bin_str))
      << "Failed to parse " << path.string();
  return message;
}

template <typename ProtobufT>
ProtobufT ReadProtobufFromResource(
    const farm_ng_proto::tractor::v1::Resource& resource) {
  if (ContentTypeProtobufJson<ProtobufT>() == resource.content_type()) {
    return ReadProtobufFromJsonFile<ProtobufT>(GetBlobstoreRoot() /
                                               resource.path());
  }
  if (ContentTypeProtobufBinary<ProtobufT>() == resource.content_type()) {
    return ReadProtobufFromBinaryFile<ProtobufT>(GetBlobstoreRoot() /
                                                 resource.path());
  }
  throw std::runtime_error(
      std::string("The content_type doesn't match expected: ") +
      ContentTypeProtobufJson<ProtobufT>() + " or " +
      ContentTypeProtobufBinary<ProtobufT>() +
      " instead : " + resource.content_type());
}
}  // namespace farm_ng

#endif
