TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Install databases, templates & substitutions like this
DB += drvAsynIseghalServiceVersion.db
DB += drvAsynIseghalService.db
DB += system-items.template
DB += can-items.template
DB += crate-items.template
DB += module-items.template
DB += channel-items.template

# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_TEMPLATE = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE
