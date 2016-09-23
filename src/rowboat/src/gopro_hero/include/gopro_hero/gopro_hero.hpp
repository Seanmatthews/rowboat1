#ifndef GOPRO_HERO_HPP_
#define GOPRO_HERO_HPP_

#include <string>
#include <map>
#include <typeinfo>

//#include <curlpp/cURLpp.hpp>
//#include <curlpp/Easy.hpp>
//#include <curlpp/Options.hpp>
#include <curl/curl.h>
#include <json/json.h>
#include <json/reader.h>
#include <sstream>
#include <iomanip>
#include <string>

#include "gopro_hero/gopro_hero_commands.hpp"


namespace rowboat1 {

    class GoProHero {
    public:

        enum class Mode {
            VIDEO,
            PHOTO,
            MULTISHOT
        };

        GoProHero() {
            base_ = GoProHeroCommands::commandBase();
            mode_ = GoProHeroCommands::Mode::Video;
        }
        
        ~GoProHero() {}

        std::string zeroPaddedIntString(std::string num, int pad) {
            std::ostringstream ss;
            ss << std::setw(pad) << std::setfill('0') << num;
            return ss.str()
        }

        // Lazy get -- move boilerplate to util function
        void getCurrentImages(std::vector<std::vector<char> >& images, long timeout = 1) {
            Json::Value root;
            Json::Reader reader;
            CURL* curl = curl_easy_init();
            CURLcode code(CURLE_FAILED_INIT);
            std::string buffer;
            
            if (curl)
            {
                if (CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_URL, "http://10.5.5.9/gp/gpMediaList"))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &curlWriteCallback))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout))
                    && CURLE_OK == (code = curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer)))
                {
                    code = curl_easy_perform(curl);
                }

                curl_easy_cleanup(curl);
            }

            if (!buffer.empty() && reader.parse(buffer, root))
            {
                const Json::Value media = root["media"][0]["fs"];
                const Json::Value lastVal = media[media.size() - 1];

                // TODO Check that it's a JPG
                
                int startNum = std::stoi(lastVal["b"]);
                int endNum = std::stoi(lastVal["l"]);
                for (int i=startNum; i<=endNum; ++i)
                {
                    std::string filename =
                        zeroPaddedIntString(lastVal["g"], 3) +
                        zeroPaddedIntString(lastVal["b"], 4) + ".JPG";
                    std::vector<char> image;
                    getImage(filename, image);
                    capturedImages_.push_back(image);
                }
                                
            }
        }

        // Is this correct? Setting mode == turning mode on?
        void setMode(Mode m) {
            mode_ = m;
            switch (m) {
            case GoProHeroCommands::Mode::VIDEO: sendSetting("10/1"); break;
            case GoProHeroCommands::Mode::PHOTO: sendSetting("21/1"); break;
            case GoProHeroCommands::Mode::MULTISHOT: sendSetting("34/1"); break;
            default: break;
            }
        }

        // Global functions
        void shutter(bool on) { sendCommand("shutter?p=" + (on ? "1" : "0")); }
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
        void photoResolution(PhotoResolution p) { sendModalSetting(p); }
        
            
    private:

        template<typename T>
        void sendModalSetting(T s) {
            switch (mode_) {
            case Mode::VIDEO:
            {
                auto it = GoProHeroCommands::videoModeVals.find(typeid(T).name());
                if (it != GoProHeroCommands::videoModeVals.end())
                    sendSetting(it->second + GoProHero::to_string(s));
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
            curlpp::Cleanup cleaner;
            curlpp::Easy req;
            req.setOpt<Url>(s);
            req.perform();
            std::cout << s << std::endl;
            return true;
        }
        
        const std::string base_;
        Mode mode_;
        std::vector<std::vector<char> > capturedImages_;
    };
}

#endif
