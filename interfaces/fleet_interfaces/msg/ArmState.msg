string STATE_IDLE="IDLE"
string STATE_PICKING="PICKING"
# Inside the basket space. arm1 and arm2 must never both report this at once.
string STATE_PLACING="PLACING"
string STATE_SWAPPING="SWAPPING"
string STATE_ERROR="ERROR"

string arm_id
string state
string detail
builtin_interfaces/Time stamp
