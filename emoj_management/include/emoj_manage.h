#ifndef __EMOJ_MANAGE_H
#define __EMOJ_MANAGE_H

#include "lvgl_base.h"

#include <memory>

namespace emoj
{

// Abstract Class for Emoj Displaying
template<typename Derived>
class Emoj
{
public:
	void show()
	{
		return static_cast<Derived*>(this)->show();
	}
};

// Factory Class
class EmojFactory
{
public:
        template<typename Derived>
        std::unique_ptr<Emoj<Derived>> createEmoj()
        {
                return std::make_unique<Emoj<Derived>>();
        }
};


// 1. happy
class Happy : public Emoj<Happy>
{
public:
	void show() // static override : CRTP
	{
		std::cout<<" Happy Emoj Display ."<<std::endl;	
		
		return lvgl::Lvgl::instance().showImgs( "S:../images/happy_", 7, 100 );
	}

};

// 2. blink
class Blink : public Emoj<Blink>
{
public:
        void show() // static override : CRTP
        {
                std::cout<<" Blink Emoj Display ."<<std::endl;

                return lvgl::Lvgl::instance().showImgs( "S:../images/blink_", 8, 150 );
        }
};


// 3. Mad
class Mad : public Emoj<Mad>
{
public:
        void show() // static override : CRTP
        {
                std::cout<<" Mad Emoj Display ."<<std::endl;

                return lvgl::Lvgl::instance().showImgs( "S:../images/mad_", 7, 100 );
        }
};

// 4. sleep
class Sleep : public Emoj<Sleep>
{
public:
        void show() // static override : CRTP
        {
                std::cout<<" Sleep Emoj Display ."<<std::endl;

                return lvgl::Lvgl::instance().showImgs( "S:../images/sleep_", 8, 100 );
        }
};

// 5. drink coeff
class Coeff : public Emoj<Coeff>
{
public:
        void show() // static override : CRTP
        {
                std::cout<<" Coeff Emoj Display ."<<std::endl;

                return lvgl::Lvgl::instance().showImgs( "S:../images/coeff_", 8, 100 );
        }
};

// 6. read book
class Book : public Emoj<Book>
{
public:
        void show() // static override : CRTP
        {
                std::cout<<" Book Emoj Display ."<<std::endl;

                return lvgl::Lvgl::instance().showImgs( "S:../images/book_", 8, 100 );
        }
};


}

#endif
