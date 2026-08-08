#
# Emits the GenICam XML that the client downloads and parses.
#
# Two rules govern what goes in here, and both are about what the client
# supplies for itself:
#
#   1. No transport layer nodes. Aravis injects defaults for every GevSCP*,
#      GevSCDA, DeviceVendorName and friend, and skips any name already
#      present in the document. Declaring one of them replaces working
#      plumbing with ours.
#
#   2. Never emit GevSCPSFireTestPacket. Its presence makes the client
#      binary search for a packet size by firing test packets, and this
#      device does not answer them.
#
# The schema version is 1.0.1 rather than something newer on purpose: below
# 1.1.0 the client uses READ_REGISTER for 4 byte accesses, which is what
# every other GigE Vision device does and what the reference implementation
# is tested against.
#

import xml.etree.ElementTree as ElementTree
from xml.sax.saxutils import escape

from .features import (CommandFeature, EnumFeature, FloatFeature, IntFeature,
                       StringFeature)

SCHEMA_NS = "http://www.genicam.org/GenApi/Version_1_0"


def _reg_name(feature):
    return feature.name + "Reg"


def _common_reg_body(feature, out, indent="\t\t"):
    out.append("%s<Address>0x%x</Address>" % (indent, feature.address))
    out.append("%s<Length>%d</Length>" % (indent, feature.size))
    out.append("%s<AccessMode>%s</AccessMode>" % (indent, feature.access))
    out.append("%s<pPort>Device</pPort>" % indent)


def _emit_feature(feature, out):
    name = feature.name
    reg = _reg_name(feature)
    description = escape(feature.description or name)

    if isinstance(feature, IntFeature):
        out.append('\t<Integer Name="%s" NameSpace="Custom">' % name)
        out.append("\t\t<Description>%s</Description>" % description)
        out.append("\t\t<pValue>%s</pValue>" % reg)
        out.append("\t\t<Min>%d</Min>" % feature.min)
        out.append("\t\t<Max>%d</Max>" % feature.max)
        out.append("\t\t<Inc>%d</Inc>" % feature.inc)
        if feature.unit:
            out.append("\t\t<Unit>%s</Unit>" % escape(feature.unit))
        out.append("\t</Integer>")
        out.append('\t<IntReg Name="%s" NameSpace="Custom">' % reg)
        _common_reg_body(feature, out)
        out.append("\t\t<Sign>Unsigned</Sign>")
        out.append("\t\t<Endianess>BigEndian</Endianess>")
        out.append("\t</IntReg>")

    elif isinstance(feature, FloatFeature):
        out.append('\t<Float Name="%s" NameSpace="Custom">' % name)
        out.append("\t\t<Description>%s</Description>" % description)
        out.append("\t\t<pValue>%s</pValue>" % reg)
        out.append("\t\t<Min>%s</Min>" % repr(float(feature.min)))
        out.append("\t\t<Max>%s</Max>" % repr(float(feature.max)))
        if feature.unit:
            out.append("\t\t<Unit>%s</Unit>" % escape(feature.unit))
        out.append("\t</Float>")
        out.append('\t<FloatReg Name="%s" NameSpace="Custom">' % reg)
        _common_reg_body(feature, out)
        out.append("\t\t<Endianess>BigEndian</Endianess>")
        out.append("\t</FloatReg>")

    elif isinstance(feature, EnumFeature):
        out.append('\t<Enumeration Name="%s" NameSpace="Custom">' % name)
        out.append("\t\t<Description>%s</Description>" % description)
        for entry_name, value in feature.entries.items():
            out.append('\t\t<EnumEntry Name="%s" NameSpace="Custom">'
                       % escape(entry_name))
            out.append("\t\t\t<Value>%d</Value>" % value)
            out.append("\t\t</EnumEntry>")
        out.append("\t\t<pValue>%s</pValue>" % reg)
        out.append("\t</Enumeration>")
        out.append('\t<IntReg Name="%s" NameSpace="Custom">' % reg)
        _common_reg_body(feature, out)
        out.append("\t\t<Sign>Unsigned</Sign>")
        out.append("\t\t<Endianess>BigEndian</Endianess>")
        out.append("\t</IntReg>")

    elif isinstance(feature, CommandFeature):
        out.append('\t<Command Name="%s" NameSpace="Custom">' % name)
        out.append("\t\t<Description>%s</Description>" % description)
        out.append("\t\t<pValue>%s</pValue>" % reg)
        out.append("\t\t<CommandValue>%d</CommandValue>" % feature.command_value)
        out.append("\t</Command>")
        out.append('\t<IntReg Name="%s" NameSpace="Custom">' % reg)
        _common_reg_body(feature, out)
        out.append("\t\t<Sign>Unsigned</Sign>")
        out.append("\t\t<Endianess>BigEndian</Endianess>")
        out.append("\t</IntReg>")

    elif isinstance(feature, StringFeature):
        out.append('\t<StringReg Name="%s" NameSpace="Custom">' % name)
        out.append("\t\t<Description>%s</Description>" % description)
        _common_reg_body(feature, out)
        out.append("\t</StringReg>")

    else:
        raise TypeError("unsupported feature type %r" % type(feature))


def build_xml(feature_set, model_name, vendor_name, tooltip=None):
    """
    Returns the GenICam XML as bytes.
    """
    categories = feature_set.categories()

    out = []
    out.append('<?xml version="1.0" encoding="utf-8"?>')
    out.append("")
    out.append("<RegisterDescription")
    out.append('\tModelName="%s"' % escape(model_name))
    out.append('\tVendorName="%s"' % escape(vendor_name))
    out.append('\tStandardNameSpace="None"')
    out.append('\tSchemaMajorVersion="1"')
    out.append('\tSchemaMinorVersion="0"')
    out.append('\tSchemaSubMinorVersion="1"')
    out.append('\tMajorVersion="1"')
    out.append('\tMinorVersion="0"')
    out.append('\tSubMinorVersion="0"')
    out.append('\tToolTip="%s"' % escape(tooltip or model_name))
    out.append('\tProductGuid="0"')
    out.append('\tVersionGuid="0"')
    out.append('\txmlns="%s"' % SCHEMA_NS)
    out.append('\txmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"')
    out.append('\txsi:schemaLocation="%s GenApiSchema.xsd">' % SCHEMA_NS)
    out.append("")

    out.append('\t<Category Name="Root" NameSpace="Standard">')
    for category in categories:
        out.append("\t\t<pFeature>%s</pFeature>" % category)
    out.append("\t</Category>")
    out.append("")

    for category in categories:
        out.append('\t<Category Name="%s" NameSpace="Custom">' % category)
        for feature in feature_set.features:
            if feature.category == category:
                out.append("\t\t<pFeature>%s</pFeature>" % feature.name)
        out.append("\t</Category>")
        out.append("")

    for feature in feature_set.features:
        _emit_feature(feature, out)
        out.append("")

    # Every pPort in the document resolves here; this node is the bridge
    # between GenICam register access and GVCP.
    out.append('\t<Port Name="Device" NameSpace="Standard"/>')
    out.append("")
    out.append("</RegisterDescription>")
    out.append("")

    return "\n".join(out).encode("utf-8")


def validate_xml(xml_bytes, feature_set):
    """
    Structural checks that would otherwise only show up as a client that
    silently refuses to stream. Called from the tests, and cheap enough to
    call at startup.
    """
    root = ElementTree.fromstring(xml_bytes)
    problems = []

    tag = "{%s}" % SCHEMA_NS
    if not root.tag.endswith("RegisterDescription"):
        problems.append("root element is %r" % root.tag)

    names = set()
    for element in root.iter():
        name = element.get("Name")
        if name is None:
            continue
        names.add(name)
        if name.startswith("Gev") or name.startswith("ArvGev"):
            problems.append("%r collides with an injected transport node" % name)

    if "Device" not in names:
        problems.append("missing <Port Name=\"Device\"/>")

    # Every address in the document must match what the allocator assigned.
    for element in root.iter(tag + "IntReg"):
        _check_address(element, tag, feature_set, problems)
    for element in root.iter(tag + "FloatReg"):
        _check_address(element, tag, feature_set, problems)
    for element in root.iter(tag + "StringReg"):
        _check_address(element, tag, feature_set, problems)

    return problems


def _check_address(element, tag, feature_set, problems):
    name = element.get("Name")
    address_element = element.find(tag + "Address")
    if address_element is None:
        problems.append("%s has no <Address>" % name)
        return
    address = int(address_element.text, 0)
    if not (feature_set.arena_start <= address < feature_set.arena_end):
        problems.append("%s address 0x%x is outside the feature arena"
                        % (name, address))
        return
    feature = feature_set.lookup_address(address)
    if feature is None:
        problems.append("%s address 0x%x matches no allocated feature"
                        % (name, address))
