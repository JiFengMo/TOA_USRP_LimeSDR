# Check and emit the NR-TOA SSB TOA placeholder accelerator IP scaffold.
# Usage:
#   vivado -mode batch -source fpga/ip/ssb_toa/package_ip.tcl -tclargs fpga/ip/ssb_toa
#
# Expected outputs:
#   fpga/ip/ssb_toa/packaged/component.xml
#   fpga/ip/ssb_toa/packaged/hdl/ssb_toa_axis_stub.v
#   fpga/ip/ssb_toa/packaged/reports/ssb_toa_elab.rpt

if {$argc < 1} {
  set ip_root [file normalize [file dirname [info script]]]
} else {
  set ip_root [file normalize [lindex $argv 0]]
}

set rtl_dir [file join $ip_root rtl]
set rtl_file [file join $rtl_dir ssb_toa_axis_stub.v]

if {![file exists $rtl_file]} {
  error "Missing RTL file: $rtl_file"
}

set pkg_out [file join $ip_root packaged]
set pkg_hdl [file join $pkg_out hdl]
set pkg_reports [file join $pkg_out reports]
file mkdir $pkg_out
file mkdir $pkg_hdl
file mkdir $pkg_reports
if {[info exists ::env(TEMP)]} {
  set temp_base [file normalize $::env(TEMP)]
} else {
  set temp_base [file normalize [pwd]]
}
set pkg_work [file join $temp_base nr_toa_ssb_ip_pkg_work]
file delete -force $pkg_work
file mkdir $pkg_work
set temp_rtl_dir [file join $pkg_work rtl]
file mkdir $temp_rtl_dir
set temp_rtl_file [file join $temp_rtl_dir ssb_toa_axis_stub.v]
file copy -force $rtl_file $temp_rtl_file
create_project ssb_toa_ip_pkg $pkg_work -part xc7z020clg484-1 -force
set_property target_language Verilog [current_project]
add_files -fileset sources_1 -norecurse [list $temp_rtl_file]
set rtl_obj [get_files [list $temp_rtl_file]]
set_property file_type Verilog $rtl_obj
set_property used_in_synthesis true $rtl_obj
set_property used_in_simulation true $rtl_obj
set_property top ssb_toa_axis_stub [current_fileset]
update_compile_order -fileset sources_1

check_syntax -fileset sources_1
set rpt [open [file join $pkg_reports ssb_toa_elab.rpt] w]
puts $rpt "Vivado syntax check passed for ssb_toa_axis_stub"
puts $rpt "Part: xc7z020clg484-1"
puts $rpt "Top: ssb_toa_axis_stub"
close $rpt

file copy -force $rtl_file [file join $pkg_hdl ssb_toa_axis_stub.v]

set component_file [file join $pkg_out component.xml]
set fp [open $component_file w]
puts $fp {<?xml version="1.0" encoding="UTF-8"?>}
puts $fp {<spirit:component xmlns:spirit="http://www.spiritconsortium.org/XMLSchema/SPIRIT/1685-2009">}
puts $fp {  <spirit:vendor>user.org</spirit:vendor>}
puts $fp {  <spirit:library>nr_toa</spirit:library>}
puts $fp {  <spirit:name>ssb_toa_axis_stub</spirit:name>}
puts $fp {  <spirit:version>0.1</spirit:version>}
puts $fp {  <spirit:description>AXI-Stream scaffold for NR SSB TOA FPGA bring-up. Run Vivado IP Packager or ADI project integration to infer final bus metadata.</spirit:description>}
puts $fp {  <spirit:fileSets>}
puts $fp {    <spirit:fileSet>}
puts $fp {      <spirit:name>xilinx_verilogsynthesis</spirit:name>}
puts $fp {      <spirit:file>}
puts $fp {        <spirit:name>hdl/ssb_toa_axis_stub.v</spirit:name>}
puts $fp {        <spirit:fileType>verilogSource</spirit:fileType>}
puts $fp {      </spirit:file>}
puts $fp {    </spirit:fileSet>}
puts $fp {  </spirit:fileSets>}
puts $fp {</spirit:component>}
close $fp

puts "Checked and emitted NR TOA SSB IP scaffold at $pkg_out"
