#ifndef LP_PLUGIN_SIMULATOR_GLOBAL_H
#define LP_PLUGIN_SIMULATOR_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(LP_PLUGIN_SIMULATOR_LIBRARY)
#  define LP_PLUGIN_SIMULATOR_EXPORT Q_DECL_EXPORT
#else
#  define LP_PLUGIN_SIMULATOR_EXPORT Q_DECL_IMPORT
#endif

#endif // LP_PLUGIN_SIMULATOR_GLOBAL_H