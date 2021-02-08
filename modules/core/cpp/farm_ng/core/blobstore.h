#ifndef FARM_NG_BLOBSTORE_H_
#define FARM_NG_BLOBSTORE_H_

#include <map>
#include <stdexcept>

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include <boost/filesystem.hpp>
#include "glog/logging.h"

#include "farm_ng/core/resource.pb.h"

namespace farm_ng {
namespace core {

boost::filesystem::path GetBlobstoreRoot();
boost::filesystem::path GetBucketRelativePath(Bucket id);
boost::filesystem::path GetBucketAbsolutePath(Bucket id);

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
farm_ng::core::Resource WriteProtobufAsJsonResource(Bucket id,
                                                    const std::string& path,
                                                    const ProtobufT& message) {
  farm_ng::core::Resource resource;
  resource.set_content_type(ContentTypeProtobufJson<ProtobufT>());

  boost::filesystem::path write_path =
      MakePathUnique(GetBucketAbsolutePath(id), path);
  write_path += ".json";
  resource.set_path((GetBucketRelativePath(id) / write_path).string());
  WriteProtobufToJsonFile(GetBucketAbsolutePath(id) / write_path, message);
  return resource;
}

template <typename ProtobufT>
farm_ng::core::Resource WriteProtobufAsBinaryResource(
    Bucket id, const std::string& path, const ProtobufT& message) {
  farm_ng::core::Resource resource;
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
ProtobufT ReadProtobufFromResource(const farm_ng::core::Resource& resource) {
  boost::filesystem::path resource_path(resource.path());
  if (!boost::filesystem::exists(resource_path)) {
    resource_path = GetBlobstoreRoot() / resource_path;
  }
  if (ContentTypeProtobufJson<ProtobufT>() == resource.content_type()) {
    return ReadProtobufFromJsonFile<ProtobufT>(resource_path);
  }
  if (ContentTypeProtobufBinary<ProtobufT>() == resource.content_type()) {
    return ReadProtobufFromBinaryFile<ProtobufT>(resource_path);
  }
  throw std::runtime_error(
      std::string("The content_type doesn't match expected: ") +
      ContentTypeProtobufJson<ProtobufT>() + " or " +
      ContentTypeProtobufBinary<ProtobufT>() +
      " instead : " + resource.content_type());
}

}  // namespace core
}  // namespace farm_ng

#endif
