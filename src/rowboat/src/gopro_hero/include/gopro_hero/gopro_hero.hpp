#ifndef GOPRO_HERO_HPP_
#define GOPRO_HERO_HPP_

#include <string>
#include <map>
#include <typeinfo>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <sstream>
#include <iomanip>


#include "gopro_hero_commands.hpp"


namespace rowboat1 {

    class GoProHero {
    public:

        enum class Mode {
            VIDEO,
            PHOTO,
            MULTISHOT
        };

        GoProHero() :
            commsTimeoutSeconds_(2) {
            //base_ = GoProHeroCommands::commandBase();
            mode_ = Mode::PHOTO;
            curl_global_init(CURL_GLOBAL_ALL);
        }
        
        ~GoProHero() {
            curl_global_cleanup();
        }

        std::string zeroPaddedIntString(std::string num, int pad) {
            std::ostringstream ss;
            ss << std::setw(pad) << std::setfill('0') << num;
            return ss.str();
        }



        // Lazy get -- move boilerplate to util function
        void getCurrentImages(std::vector<std::vector<char> >& images, long timeout = 1) {
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
                    std::vector<char> image;
                    curlGetImage(filename, image);
                    capturedImages_.push_back(image);
                }
                                
            }
        }

        // Is this correct? Setting mode == turning mode on?
        void setMode(Mode m) {
            mode_ = m;
            switch (m) {
            case Mode::VIDEO: sendSetting("10/1"); break;
            case Mode::PHOTO: sendSetting("21/1"); break;
            case Mode::MULTISHOT: sendSetting("34/1"); break;
            default: break;
            }
        }

        // Global functions
        void shutter(bool on) { sendCommand("shutter?p=" + std::to_string((on ? 1 : 0))); }
/*
        void orientation(Orientation o) { sendSetting("52/" + GoProHeroCommands::to_string(o)); }
        void ledBlink(LEDBlink b) { sendSetting("55/" + GoProHeroCommands::to_string(b)); }
        void beep(Beep b) { sendSetting("56/" + GoProHeroCommands::to_string(b)); }
        void lcdDisplay(bool on) { sendSetting("72/" + (on ? "1" : "0")); }
        void onScreenDisplay(bool on) { sendSetting("58/" + (on ? "1" : "0")); }
        void lcdBrightness(LCDBrightness b) { sendSetting("49/" + GoProHeroCommands::to_string(b)); }
        void lcdLock(bool on) { sendSetting("50/" + (on ? "1" : "0")); }
        void lcdSleepTimeout(LCDSleepTimeout t) { sendSetting("51/" + GoProHeroCommands::to_string(t)); }
        void autoOffTime(AutoOffTime a) { sendSetting("59/" + GoProHeroCommands::to_string(a)); }


        // Single mode functions
        void streamBitRate(StreamBitRate s) { sendSetting("62/" + GoProHeroCommands::to_string(s)); }
        void streamWindowSize(StreamWindowSize s) { sendSetting("64/" + GoProHeroCommands::to_string(s)); }
*/
        
        // TODO
        // tag moment
        // pair with RC
        // locate
        // delete file
        // delete last file
        // reformat sd card

        // Mode-specific
        void whiteBalance(WhiteBalance w) { sendModalSetting(w); }
        void color(Color c) { sendModalSetting(c); }
        void isoLimit(ISOLimit i) { sendModalSetting(i); }
        void isoMin(ISOMin i) { sendModalSetting(i); }
        void sharpness(Sharpness s) { sendModalSetting(s); }
        void ev(EV e) { sendModalSetting(e); }
        void exposure(Exposure e) { sendModalSetting(e); }
//        void photoResolution(PhotoResolution p) { sendModalSetting(p); }
        
            
    private:
        template<typename T>
        void sendModalSetting(T s) {
            switch (mode_) {
            case Mode::VIDEO:
            {
                auto it = GoProHeroCommands::videoModeVals.find(typeid(T).name());
                if (it != GoProHeroCommands::videoModeVals.end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            case Mode::PHOTO:
            {
                auto it = GoProHeroCommands::photoModeVals.find(typeid(T).name());
                if (it != GoProHeroCommands::photoModeVals.end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            case Mode::MULTISHOT:
            {
                auto it = GoProHeroCommands::multiModeVals.find(typeid(T).name());
                if (it != GoProHeroCommands::multiModeVals.end())
                    sendSetting(it->second + GoProHeroCommands::to_string(s));
                break;
            }
            default:
                break;
            }
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

    public:
        bool curlGetImage(const std::string url, std::vector<char>& image) {
            std::ostringstream oss;
            if (curlRequestUrl(url, oss))
            {
                std::string s = oss.str();
                std::copy(s.begin(), s.end(), std::back_inserter(image));
                return true;
            }
            return false;
        }
        

        static size_t curlWriteCallback(void *buf, size_t size, size_t nmemb, void *userp) {
            if(userp)
            {
                std::ostream& os = *static_cast<std::ostream*>(userp);
                std::streamsize len = size * nmemb;
                if(os.write(static_cast<char*>(buf), len)) return len;
            }
            return 0;
        }

        bool curlGetText(const std::string url, std::string& text) {
            std::ostringstream oss;
            if (curlRequestUrl(url, oss))
            {
                text = oss.str();
                return true;
            }
            return false;
        }
        
        bool curlRequestUrl(const std::string url, std::ostream& os) {
            CURL* curl = curl_easy_init();
            CURLcode code(CURLE_FAILED_INIT);

            if (curl)
            {
                if (CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
                                                            &GoProHero::curlWriteCallback))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_FILE, &os))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_URL, url.c_str())))
                {
                    code = curl_easy_perform(curl);
                }
                curl_easy_cleanup(curl);
            }
            return code == CURLE_OK;
        }

        
        
        const std::string base_ = GoProHeroCommands::commandBase();
        Mode mode_;
        std::vector<std::vector<char> > capturedImages_;
        long commsTimeoutSeconds_;
    };
}

#endif
