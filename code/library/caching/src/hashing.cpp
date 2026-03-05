#include "hashing.hpp"

#include <openssl/evp.h>

#include <format>

namespace reprojection::caching {

std::string Sha256(std::string_view input) {
    std::array<unsigned char, EVP_MAX_MD_SIZE> hash;
    unsigned int hash_length{0};

    if (EVP_Digest(std::data(input), std::size(input), std::data(hash), &hash_length, EVP_sha256(), nullptr) != 1) {
        throw std::runtime_error("EVP_Digest failed");  // LCOV_EXCL_LINE
    }

    std::string result;
    result.reserve(hash_length * 2);
    for (unsigned int i{0}; i < hash_length; ++i) {
        result += std::format("{:02x}", hash[i]);
    }

    return result;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::caching