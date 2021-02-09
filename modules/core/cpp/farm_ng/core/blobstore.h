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
namespace fs = boost::filesystem;

fs::path GetBlobstoreRoot();
fs::path GetBucketRelativePath(Bucket id);
fs::path GetBucketAbsolutePath(Bucket id);

fs::path NativePathFromResourcePath(const Resource& resource);

// Returns a bucket-relative path guaranteed to be unique, with a parent
// directory created if necessary.
fs::path MakePathUnique(fs::path root, fs::path path);

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

template <typename ProtobufT>
Resource ProtobufJsonResource(const fs::path& path) {
  Resource resource;
  resource.set_path(path.string());
  resource.set_content_type(ContentTypeProtobufJson<ProtobufT>());
  return resource;
}

template <typename ProtobufT>
Resource ProtobufBinaryResource(const fs::path& path) {
  Resource resource;
  resource.set_path(path.string());
  resource.set_content_type(ContentTypeProtobufBinary<ProtobufT>());
  return resource;
}

// Construct a Resource pointing to an event log on disk.
Resource EventLogResource(const fs::path& path);

void WriteProtobufToJsonFile(const fs::path& path,
                             const google::protobuf::Message& proto);

void WriteProtobufToBinaryFile(const fs::path& path,
                               const google::protobuf::Message& proto);

template <typename ProtobufT>
farm_ng::core::Resource WriteProtobufAsJsonResource(Bucket id,
                                                    const std::string& path,
                                                    const ProtobufT& message) {
  farm_ng::core::Resource resource;
  resource.set_content_type(ContentTypeProtobufJson<ProtobufT>());

  fs::path write_path =
      MakePathUnique(GetBucketAbsolutePath(id), path + ".json");
  resource.set_path((GetBucketRelativePath(id) / write_path).string());
  WriteProtobufToJsonFile(NativePathFromResourcePath(resource), message);
  return resource;
}

template <typename ProtobufT>
farm_ng::core::Resource WriteProtobufAsBinaryResource(
    Bucket id, const std::string& path, const ProtobufT& message) {
  farm_ng::core::Resource resource;
  resource.set_content_type(ContentTypeProtobufBinary<ProtobufT>());

  fs::path write_path = MakePathUnique(GetBucketAbsolutePath(id), path + ".pb");
  resource.set_path((GetBucketRelativePath(id) / write_path).string());
  WriteProtobufToBinaryFile(NativePathFromResourcePath(resource), message);
  return resource;
}

template <typename ProtobufT>
ProtobufT ReadProtobufFromJsonFile(const fs::path& path) {
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
ProtobufT ReadProtobufFromBinaryFile(const fs::path& path) {
  VLOG(2) << "Loading (binary proto) : " << path.string();
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
  CHECK_EQ(resource.payload_case(),
           farm_ng::core::Resource::PayloadCase::kPath);
  fs::path resource_path(NativePathFromResourcePath(resource));
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
