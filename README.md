# xiaozhi-esp32 本地增强版

本地版本基于 [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) 开源项目，进行了如下增强：

## 本地优势

- 新增在线播放音乐功能：支持通过关键词联网搜索、下载并播放网络音乐，自动提示搜索和播放状态。
- 支持 mp3 音频解码，兼容更多音乐格式。
- 保留原有全部语音助手功能。

---

## 本地实现的主要接口

### 1. PlayOnlineMusic(const std::string& keyword)

通过关键词联网搜索、下载并播放网络音乐。

### 2. DownloadMusic(const std::string& keyword)

根据关键词请求音乐API，下载音乐文件并返回文件名。

### 3. PlayMusicWithFilename(const std::string& filename)

根据指定文件名播放已下载的音乐文件。

这些接口位于 `main/application.h` 和 `main/application.cc`，为本地在线播放音乐功能的核心实现。

本地增强版适合需要语音助手+在线音乐播放一体化体验的用户。

---

开源基础项目地址：[https://github.com/78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)

## 使用说明

1. 按照官方文档配置 ESP-IDF 环境。
2. 编译并烧录固件到支持的 ESP32 开发板。
3. 如需使用在线播放功能，请确保设备联网，并配置好音乐 API 服务器地址（可在 `wifi` 设置项或 `CONFIG_OTA_URL` 中指定）。

---

## 致谢

- 本地增强版基于 [Luoqi2003/xiaozhi-esp32-music](https://github.com/Luoqi2003/xiaozhi-esp32-music) 开发。
- 感谢所有开源贡献者！
