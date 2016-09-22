#ifndef GOPRO_HERO_HPP_
#define GOPRO_HERO_HPP_

#include <string>
#include <map>
#include <typeinfo>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>


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
        }
        
        ~GoProHero() {}


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

        
        void sendSetting(std::string s) {
            send(base_ + "setting/" + s);
        }

        
        void sendCommand(std::string s) {
            send(base_ + "command/" + s);
        }

        // TODO accept and parse output for success/failure--
        // maybe have this func return actual string, then sendCommand/sendSetting
        // can parse based on their respectie expected outputs.
        // TODO catch exceptions
        bool send(std::string s) {
//            curlpp::Cleanup cleaner;
//            curlpp::Easy req;
//            req.setOpt<Url>(s);
//            req.perform();
            std::cout << s << std::endl;
            return true;
        }
        
        const std::string base_;
        Mode mode_;

        const std::map<std::string, std::string> videoModeVal_ = {
            {typeid(WhiteBalance).name(), "11/"},
            {typeid(Color).name(), "12/"},
            {typeid(ISOLimit).name(), "13/"},
            {typeid(Sharpness).name(), "14/"},
            {typeid(EV).name(), "15/"}
        };
    
    };
}

#endif
