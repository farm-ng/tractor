#include "farm_ng/core/blobstore.h"

namespace farm_ng {
namespace core {

fs::path GetBlobstoreRoot() {
  fs::path root;
  if (const char* env_root = std::getenv("BLOBSTORE_ROOT")) {
    root = env_root;
  } else {
    root = "/tmp/farm-ng-log/";
  }
  return root;
}

fs::path GetBucketRelativePath(Bucket id) {
  std::string path =
      Bucket_Name(id).substr(std::strlen("BUCKET_"), std::string::npos);
  std::transform(path.begin(), path.end(), path.begin(), ::tolower);
  return path;
}

fs::path GetBucketAbsolutePath(Bucket id) {
  return GetBlobstoreRoot() / GetBucketRelativePath(id);
}

fs::path NativePathFromResourcePath(const farm_ng::core::Resource& resource) {
  CHECK_EQ(resource.payload_case(),
           farm_ng::core::Resource::PayloadCase::kPath);
  fs::path resource_path(resource.path());
  if (fs::exists(resource_path) || resource_path.is_absolute()) {
    return resource_path;
  }
  return (GetBlobstoreRoot() / resource_path);
}

Resource EventLogResource(const fs::path& path) {
  Resource resource;
  resource.set_path(path.string());
  resource.set_content_type("application/farm_ng.eventlog.v1");
  return resource;
}

void WriteProtobufToJsonFile(const fs::path& path,
                             const google::protobuf::Message& proto) {
  google::protobuf::util::JsonPrintOptions print_options;
  print_options.add_whitespace = true;
  print_options.always_print_primitive_fields = true;
  std::string json_str;
  google::protobuf::util::MessageToJsonString(proto, &json_str, print_options);
  std::ofstream outf(path.string());
  outf << json_str;
}

void WriteProtobufToBinaryFile(const fs::path& path,
                               const google::protobuf::Message& proto) {
  std::string binary_str;
  proto.SerializeToString(&binary_str);
  std::ofstream outf(path.string(), std::ofstream::binary);
  outf << binary_str;
}

// Returns a bucket - relative path guaranteed to be unique,
//  with a parent
// directory created if necessary.
fs::path MakePathUnique(fs::path root, fs::path path) {
  fs::path out_path = path;
  int suffix = 1;
  while (fs::exists(root / out_path)) {
    out_path =
        path.replace_extension(std::to_string(suffix) + fs::extension(path));
    suffix++;
  }

  if (!fs::exists((root / out_path).parent_path())) {
    if (!fs::create_directories((root / out_path).parent_path())) {
      throw std::runtime_error(std::string("Could not create directory: ") +
                               (root / out_path).parent_path().string());
    }
  }

  return out_path;
}

}  // namespace core
}  // namespace farm_ng
