#ifndef GOPRO_HERO_HPP_
#define GOPRO_HERO_HPP_

#include <string>
#include <map>
#include <typeinfo>
#include <sstream>
#include <iomanip>
#include <iostream>

#include <curl/curl.h>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>

#include "gopro_hero_commands.hpp"


namespace rowboat1 {

    class GoProHero {
    public:

        using Mode = PrimaryMode;

        GoProHero() :
            commsTimeoutSeconds_(2),
            isStreaming_(false),
            saveOnDevice_(true) {
            mode_ = Mode::PHOTO;
            curl_global_init(CURL_GLOBAL_ALL);
        }
        
        ~GoProHero() {
            curl_global_cleanup();
        }

        bool isStreaming() {
            return isStreaming_;
        }
        
        std::string zeroPaddedIntString(std::string num, int pad) {
            std::ostringstream ss;
            ss << std::setw(pad) << std::setfill('0') << num;
            return ss.str();
        }



        // Lazy get -- move boilerplate to util function
        void currentImages(std::vector<std::vector<unsigned char> >& images, long timeout = 5) {
            Json::Value root;
            Json::Reader reader;
            std::string mediaList;

            if (!curlGetText("http://10.5.5.9/gp/gpMediaList", mediaList)) return;

            if (!mediaList.empty() && reader.parse(mediaList, root))
            {
                const Json::Value media = root["media"][0]["fs"];
                const Json::Value lastVal = media[media.size() - 1];

                // TODO Check that it's a JPG
                
                int startNum = std::stoi(lastVal["b"].asString());
                int endNum = std::stoi(lastVal["l"].asString());
                for (int i=startNum; i<=endNum; ++i)
                {
                    std::string filename =
                        zeroPaddedIntString(lastVal["g"].asString(), 3) +
                        zeroPaddedIntString(lastVal["b"].asString(), 4) + ".JPG";
                    std::vector<unsigned char> image;
                    curlGetBytes(filename, image);
                    images.push_back(image);
                }
                                
            }
        }

        // Is this correct? Setting mode == turning mode on?
        void setMode(Mode m) {
            mode_ = m;
            switch (m) {
            case Mode::VIDEO:
            {
                sendCommand("mode?p=0"); // set as mode
                sendSetting("10/1"); // turn on
                break;
            }
            case Mode::PHOTO:
            {
                sendCommand("mode?p=1");
                sendSetting("21/1");
                break;
            }
            case Mode::MULTISHOT:
            {
                sendCommand("mode?p=2");
                sendSetting("34/1");
                break;
            }
            default: break;
            }
        }

        // Global functions
        void shutter(bool on) { sendCommand("shutter?p=" + std::to_string((on ? 1 : 0))); }
        void orientation(Orientation o) { sendSetting("52/" + GoProHeroCommands::to_string(o)); }
        void ledBlink(LEDBlink b) { sendSetting("55/" + GoProHeroCommands::to_string(b)); }
        void beepVolume(BeepVolume b) { sendSetting("56/" + GoProHeroCommands::to_string(b)); }
        void lcdDisplay(bool on) { sendSetting("72/" + std::to_string(on ? 1 : 0)); }
        void onScreenDisplay(bool on) { sendSetting("58/" + std::to_string(on ? 1 : 0)); }
        void lcdBrightness(LCDBrightness b) { sendSetting("49/" + GoProHeroCommands::to_string(b)); }
        void lcdLock(bool on) { sendSetting("50/" + std::to_string(on ? 1 : 0)); }
        void lcdSleepTimeout(LCDSleepTimeout t) { sendSetting("51/" + GoProHeroCommands::to_string(t)); }
        void autoOffTime(AutoOffTime a) { sendSetting("59/" + GoProHeroCommands::to_string(a)); }
        void defaultBootMode(DefaultBootMode d) { sendSetting("53/" + GoProHeroCommands::to_string(d)); }
        void saveMediaOnDevice(bool yes) { saveOnDevice_ = yes; }
        void deleteLastTaken() { sendCommand("storage/delete/last"); }
        void deleteAllMedia() { sendCommand("storage/delete/all"); }
        void locate(bool on) { sendCommand("system/locate?p=" + std::to_string(on ? 1 : 0)); }
        void power(bool on, std::array<unsigned char, 6> mac = {}) {
            if (on) sendMagicPacket(mac);
            else sendCommand("system/sleep");
        }
        
        // Single mode functions
        void videoStreamBitRate(VideoStreamBitRate s) { sendSetting("62/" + GoProHeroCommands::to_string(s)); }
        void videoStreamWindowSize(VideoStreamWindowSize s) { sendSetting("64/" + GoProHeroCommands::to_string(s)); }
        void videoResolution(VideoResolution v) { sendSetting("2/" + GoProHeroCommands::to_string(v)); }
        void videoFrameRate(VideoFrameRate f) { sendSetting("3/" + GoProHeroCommands::to_string(f)); }
        void videoFOV(VideoFOV f) { sendSetting("4/" + GoProHeroCommands::to_string(f)); }
        void videoLowLight(bool on) { sendSetting("8/" + std::to_string(on ? 1 : 0)); }
        void videoLoopDuration(VideoLoopDuration v) { sendSetting("6/" + GoProHeroCommands::to_string(v)); }
        void videoPhotoInterval(VideoPhotoInterval v) { sendSetting("7/" + GoProHeroCommands::to_string(v)); }
        void videoTagMoment() { sendCommand("storage/tag_moment"); }
        void multiBurstRate(MultiBurstRate m) { sendSetting("29/" + GoProHeroCommands::to_string(m)); }
        void multiTimeLapseInterval(MultiTimeLapseInterval m) { sendSetting("31/" + GoProHeroCommands::to_string(m)); }
        void multiTimeLapseInterval(MultiNightLapseInterval m) { sendSetting("32/" + GoProHeroCommands::to_string(m)); }
        
        // Mode-specific settings -- depend on current mode
        void whiteBalance(WhiteBalance w) { sendModalSetting(w); }
        void color(Color c) { sendModalSetting(c); }
        void isoLimit(ISOLimit i) { sendModalSetting(i); }
        void isoMin(ISOMin i) { sendModalSetting(i); }
        void sharpness(Sharpness s) { sendModalSetting(s); }
        void ev(EV e) { sendModalSetting(e); }
        void exposure(Exposure e) { sendModalSetting(e); }
        void spotMeter(SpotMeter s) { sendModalSetting(s); }
        void photoResolution(PhotoResolution p) { sendModalSetting(p); }
        
            
    private:
        template<typename T>
        void sendModalSetting(T s) {
            switch (mode_) {
            case Mode::VIDEO:
            {
                auto it = GoProHeroCommands::videoModeVals().find(typeid(T).name());
                if (it != GoProHeroCommands::videoModeVals().end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            case Mode::PHOTO:
            {
                auto it = GoProHeroCommands::photoModeVals().find(typeid(T).name());
                if (it != GoProHeroCommands::photoModeVals().end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            case Mode::MULTISHOT:
            {
                auto it = GoProHeroCommands::multiModeVals().find(typeid(T).name());
                if (it != GoProHeroCommands::multiModeVals().end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            default:
                break;
            }
        }

        void sendMagicPacket(std::array<unsigned char, 6> mac) {
            using namespace boost::asio;

            std::array<unsigned char, 102> buf;

            for (int i=0; i<6; ++i) buf[i] = 0xFF; // 6 bytes
            for (int i=1; i<17; ++i) memcpy(&buf[i*6], &mac, 6 * sizeof(unsigned char)); // 96 bytes

            // send as UDP packet
            io_service ioService;
            ip::udp::socket socket(ioService);
            ip::udp::endpoint remoteEndpoint;
//            boost::system::error::error_code err;
            
            socket.open(ip::udp::v4());
            remoteEndpoint = ip::udp::endpoint(ip::address::from_string("10.5.5.9"), 9);
            socket.send_to(buffer(buf), remoteEndpoint); //, 0, err);
            socket.close();
        }
        
        void sendSetting(std::string s) { send(base_ + "setting/" + s); }
        void sendCommand(std::string s) { send(base_ + "command/" + s); }

        // TODO accept and parse output for success/failure--
        // maybe have this func return actual string, then sendCommand/sendSetting
        // can parse based on their respectie expected outputs.
        // TODO catch exceptions
        bool send(std::string s) {
            std::string empty;
            curlGetText(s, empty);
            return true;
        }


        bool curlGetBytes(const std::string url, std::vector<unsigned char>& image) {
            std::string s;
            if (curlRequestUrl(url, s))
            {
                std::copy(s.begin(), s.end(), std::back_inserter(image));
                return true;
            }
            return false;
        }

        
        static size_t curlWriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
            ((std::string*)userp)->append((char*)contents, size * nmemb);
            return size * nmemb;
        }

        
        bool curlGetText(const std::string url, std::string& text) {
            return curlRequestUrl(url, text);
        }
        
        bool curlRequestUrl(const std::string url, std::string& readBuffer) {
            CURL* curl = curl_easy_init();
            CURLcode res(CURLE_FAILED_INIT);

            if(curl) {
                curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &GoProHero::curlWriteCallback);
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
                curl_easy_setopt(curl, CURLOPT_TIMEOUT, commsTimeoutSeconds_);
                curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
                res = curl_easy_perform(curl);
                curl_easy_cleanup(curl);

                std::cout << readBuffer.size() << std::endl;
            }
            return CURLE_OK == res;
        }

        
        
        const std::string base_ = GoProHeroCommands::commandBase();
        Mode mode_;
        long commsTimeoutSeconds_;
        bool isStreaming_;
        bool saveOnDevice_;
    };
}

#endif
