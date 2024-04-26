#pragma once

#include "plugins/component_metadata/component_metadata.h"
#include "plugin_impl_base.h"
#include "callback_list.h"
#include "file_cache.h"
#include "http_loader.h"
#include <json/json.h>

#include <optional>
#include <set>
#include <string>

namespace mavsdk {

class MetadataComponentUris {
public:
    MetadataComponentUris() = default;
    MetadataComponentUris(std::string metadata_uri, uint32_t metadata_crc) :
        _crc_metadata_valid(true),
        _crc_metadata(metadata_crc),
        _uri_metadata(std::move(metadata_uri))
    {}
    explicit MetadataComponentUris(const Json::Value& value);

    const std::string& uri_metadata() const { return _uri_metadata; }
    const std::string& uri_metadata_fallback() const { return _uri_metadata_fallback; }
    const std::string& uri_translation() const { return _uri_translation; }
    const std::string& uri_translation_fallback() const { return _uri_translation_fallback; }

    uint32_t crc_meta_data() const { return _crc_metadata; }
    uint32_t crc_meta_data_fallback() const { return _crc_metadata_fallback; }
    bool crc_meta_data_valid() const { return _crc_metadata_valid; }
    bool crc_meta_data_fallback_valid() const { return _crc_metadata_fallback_valid; }

    bool available() const { return !_uri_metadata.empty(); }

private:
    bool _crc_metadata_valid{false};
    bool _crc_metadata_fallback_valid{false};

    uint32_t _crc_metadata{};
    uint32_t _crc_metadata_fallback{};

    std::string _uri_metadata;
    std::string _uri_metadata_fallback;
    std::string _uri_translation;
    std::string _uri_translation_fallback;
};

class MetadataComponent {
public:
    MetadataComponent() : _metadata_uris() {}
    explicit MetadataComponent(MetadataComponentUris metadata_uris) :
        _metadata_uris(std::move(metadata_uris))
    {}

    bool get_next_uri(bool& crc_valid, uint32_t& crc, bool& is_translation, std::string& uri);

    std::optional<std::filesystem::path>& current_metadata_path();

    bool is_completed() const { return _state == State::Done; }

    const std::optional<std::string>& json_metadata() const { return _json_metadata; }
    const std::optional<std::string>& json_translation() const { return _json_translation; }

private:
    enum class State {
        Init,
        Metadata,
        MetadataFallback,
        Translation,
        TranslationFallback,
        Done,
    };
    const MetadataComponentUris _metadata_uris;

    State _state{State::Init};
    std::optional<std::filesystem::path> _metadata;
    std::optional<std::filesystem::path> _translation;

    std::optional<std::string> _json_metadata; ///< the final json metadata
    std::optional<std::string> _json_translation; ///< the final json translation summary
};

class ComponentMetadataImpl : public PluginImplBase {
public:
    explicit ComponentMetadataImpl(System& system);
    explicit ComponentMetadataImpl(std::shared_ptr<System> system);
    ~ComponentMetadataImpl() override;

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    void request_component(uint32_t compid);
    void request_autopilot_component() { request_component(MAV_COMP_ID_AUTOPILOT1); }

    std::pair<ComponentMetadata::Result, ComponentMetadata::MetadataData>
    get_metadata(uint32_t compid, ComponentMetadata::MetadataType type);

    ComponentMetadata::MetadataAvailableHandle
    subscribe_metadata_available(const ComponentMetadata::MetadataAvailableCallback& callback);
    void unsubscribe_metadata_available(ComponentMetadata::MetadataAvailableHandle handle);

    static bool
    uri_is_mavlinkftp(const std::string& uri, std::string& download_path, uint8_t& target_compid);
    static std::string filename_from_uri(const std::string& uri);

private:
    struct MavlinkComponent {
        std::map<COMP_METADATA_TYPE, MetadataComponent> components;
        std::optional<ComponentMetadata::Result> result{}; ///< set once all types completed
    };

    void init_object();
    void receive_component_metadata(
        MavlinkCommandSender::Result result, const mavlink_message_t& message);
    void retrieve_metadata(uint8_t compid, COMP_METADATA_TYPE type);
    static std::string
    get_file_cache_tag(uint8_t compid, int comp_info_type, uint32_t crc, bool is_translation);
    std::optional<std::filesystem::path>
    extract_and_cache_file(const std::filesystem::path& path, const std::string& file_cache_tag);
    void request_componenents();
    void on_all_types_completed(uint8_t compid);

    void handle_metadata_type_completed(uint8_t compid, COMP_METADATA_TYPE type);
    void parse_component_metadata_general(uint8_t compid, const std::string& json_metadata);

    static std::optional<ComponentMetadata::MetadataType>
    get_metadata_type(COMP_METADATA_TYPE type);

    std::set<uint8_t> _components_to_request;
    bool _is_enabled{false};

    std::mutex _notification_callbacks_mutex{}; ///< Protects access to _notification_callbacks
    CallbackList<ComponentMetadata::MetadataUpdate> _notification_callbacks;

    const std::optional<std::string>
        _translation_locale{}; ///< optional locale in the form of "language_country", e.g. de_DE
    std::recursive_mutex _mavlink_components_mutex{}; ///< Protects access to _mavlink_components
    std::map<uint8_t, MavlinkComponent> _mavlink_components{};
    std::optional<FileCache> _file_cache{};
    std::filesystem::path _tmp_download_path{};
    bool _verbose_debugging{false};

#if BUILD_WITHOUT_CURL != 1
    HttpLoader _http_loader;
#endif
};

} // namespace mavsdk
