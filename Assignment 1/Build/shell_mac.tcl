# NOTES: Comments must start on a new line.


# Main console
source tkcon_mac.tcl
package require tkcon


#------------------------------------------------------
#  The console doesn't exist yet.  If we create it
#  with show/hide, then it flashes on the screen.
#  So we cheat, and call tkcon internals to create
#  the console and customize it to our application.
#------------------------------------------------------
set tkcon::PRIV(showOnStartup) 0
set tkcon::PRIV(root) .console
set tkcon::PRIV(protocol) {tkcon hide}
set tkcon::OPT(exec) ""
tkcon::Init
tkcon title "CS 174C/274C Template Code"

package require tkcon
set ::tkcon::PRIV(root) .console  ;# optional---defaults to .tkcon

#tkcon attach main
tkcon show



