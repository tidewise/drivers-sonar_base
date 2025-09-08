#include <boost/test/unit_test.hpp>
#include <sonar_base/Dummy.hpp>

using namespace sonar_base;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    sonar_base::DummyClass dummy;
    dummy.welcome();
}
