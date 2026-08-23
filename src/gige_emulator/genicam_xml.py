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

import io
import xml.etree.ElementTree as ElementTree
import zipfile
from xml.sax.saxutils import escape

from .features import (SFNC_CATEGORIES, SFNC_ENUM_ENTRIES,
                       SFNC_FEATURES, CommandFeature, EnumFeature,
                       FloatFeature, IntFeature, StringFeature)

SCHEMA_NS = "http://www.genicam.org/GenApi/Version_1_0"


def _reg_name(feature):
    return feature.name + "Reg"


def _common_reg_body(feature, out, indent="\t\t"):
    out.append("%s<Address>0x%x</Address>" % (indent, feature.address))
    out.append("%s<Length>%d</Length>" % (indent, feature.size))
    out.append("%s<AccessMode>%s</AccessMode>" % (indent, feature.access))
    out.append("%s<pPort>Device</pPort>" % indent)
    # The invalidators go on the *register* node, not on the feature node
    # above it, and that placement is the whole point. The cached value lives
    # in the register; invalidating the feature alone leaves the register
    # cache intact and the client serves the same stale number back from it.
    #
    # After pPort, and that is not cosmetic. The schema declares a register's
    # children as a sequence -- address, length, access mode, port, then the
    # invalidators, then whatever the specific register type adds -- and a
    # strict parser stops at the first element out of place. Aravis reads
    # this file either way; GenApi does not, so with these emitted first
    # VimbaX refused every camera that had one with "expected element not
    # encountered" and an unhelpful InternalFault at the API.
    for name in feature.invalidated_by:
        out.append("%s<pInvalidator>%sReg</pInvalidator>" % (indent, name))


def _bound(feature, which, out):
    """
    Emit <Min>/<Max> or <pMin>/<pMax>, never both -- GenICam takes one form
    or the other, and a node carrying both is not valid against the schema.
    """
    pointer = getattr(feature, "p_" + which.lower(), None)
    if pointer:
        out.append("\t\t<p%s>%s</p%s>" % (which, pointer, which))
        return
    value = getattr(feature, which.lower())
    if isinstance(feature, FloatFeature):
        out.append("\t\t<%s>%s</%s>" % (which, repr(float(value)), which))
    else:
        out.append("\t\t<%s>%d</%s>" % (which, value, which))


def _namespace(name, vocabulary=SFNC_FEATURES):
    """
    Standard when the naming convention defines this name, Custom when it is
    this device's own.

    Only the feature nodes get this. The <IntReg>/<FloatReg> behind each one
    is named here ("WidthReg"), so those really are custom -- the vendor XML
    this was checked against does not put a NameSpace on its register nodes
    at all.
    """
    return "Standard" if name in vocabulary else "Custom"


def _emit_feature(feature, out):
    name = feature.name
    reg = _reg_name(feature)
    description = escape(feature.description or name)

    if isinstance(feature, IntFeature):
        out.append('\t<Integer Name="%s" NameSpace="%s">' % (name, _namespace(name)))
        out.append("\t\t<Description>%s</Description>" % description)
        out.append("\t\t<pValue>%s</pValue>" % reg)
        _bound(feature, "Min", out)
        _bound(feature, "Max", out)
        out.append("\t\t<Inc>%d</Inc>" % feature.inc)
        if feature.unit:
            out.append("\t\t<Unit>%s</Unit>" % escape(feature.unit))
        out.append("\t</Integer>")
        masked = feature.lsb is not None and feature.msb is not None
        tag = "MaskedIntReg" if masked else "IntReg"
        out.append('\t<%s Name="%s" NameSpace="Custom">' % (tag, reg))
        _common_reg_body(feature, out)
        if masked:
            # The value shares its register with flag bits -- the stream
            # channel's packet size sits under a fire-test bit and a
            # do-not-fragment bit -- so a plain register node would read the
            # flags as part of the number.
            out.append("\t\t<LSB>%d</LSB>" % feature.lsb)
            out.append("\t\t<MSB>%d</MSB>" % feature.msb)
        out.append("\t\t<Sign>Unsigned</Sign>")
        out.append("\t\t<Endianess>BigEndian</Endianess>")
        out.append("\t</%s>" % tag)

    elif isinstance(feature, FloatFeature):
        out.append('\t<Float Name="%s" NameSpace="%s">' % (name, _namespace(name)))
        out.append("\t\t<Description>%s</Description>" % description)
        out.append("\t\t<pValue>%s</pValue>" % reg)
        _bound(feature, "Min", out)
        _bound(feature, "Max", out)
        if feature.unit:
            out.append("\t\t<Unit>%s</Unit>" % escape(feature.unit))
        out.append("\t</Float>")
        out.append('\t<FloatReg Name="%s" NameSpace="Custom">' % reg)
        _common_reg_body(feature, out)
        out.append("\t\t<Endianess>BigEndian</Endianess>")
        out.append("\t</FloatReg>")

    elif isinstance(feature, EnumFeature):
        out.append('\t<Enumeration Name="%s" NameSpace="%s">' % (name, _namespace(name)))
        out.append("\t\t<Description>%s</Description>" % description)
        for entry_name, value in feature.entries.items():
            out.append('\t\t<EnumEntry Name="%s" NameSpace="%s">'
                       % (escape(entry_name),
                          _namespace(entry_name, SFNC_ENUM_ENTRIES)))
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
        out.append('\t<Command Name="%s" NameSpace="%s">' % (name, _namespace(name)))
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
        out.append('\t<StringReg Name="%s" NameSpace="%s">' % (name, _namespace(name)))
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
        # NameSpace says where the *name* came from, so a category the naming
        # convention defines is Standard and an invented one is Custom.
        # Declaring a convention name as Custom is what this used to do, and
        # it claims the name is this device's own -- harmless in practice,
        # since no client checks, but it is exactly backwards.
        namespace = ("Standard" if category in SFNC_CATEGORIES else "Custom")
        out.append('\t<Category Name="%s" NameSpace="%s">'
                   % (category, namespace))
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


def zip_xml(xml_bytes, filename):
    """
    Pack the XML into a one entry zip archive, which is what the URL points
    at when compression is on.

    The client reads the blob in 512 byte chunks -- Aravis hardcodes that at
    ARV_GVCP_DATA_SIZE_MAX -- so the download costs one round trip per 512
    bytes however fast the link is. A typical feature set is around 8 kB of
    XML and deflates to under 1.5 kB, turning 16 round trips into 3. On a
    fast link that is invisible; on a slow one it is most of the time it
    takes to open the camera, because opening is round trip bound rather
    than bandwidth bound.

    Exactly one entry, because the client takes the first name in the
    archive and asks for that one back -- Aravis prepends as it walks the
    central directory, so with several files it would get the last.
    """
    buffer = io.BytesIO()
    # A fixed timestamp so the same feature set always produces the same
    # bytes; nothing reads it, and a blob that changed size between runs
    # would make the memory map depend on the clock.
    entry = zipfile.ZipInfo(filename, date_time=(1980, 1, 1, 0, 0, 0))
    entry.compress_type = zipfile.ZIP_DEFLATED
    with zipfile.ZipFile(buffer, "w") as archive:
        archive.writestr(entry, xml_bytes, compresslevel=9)
    return buffer.getvalue()


def unzip_xml(blob):
    """
    The inverse, for a client. Returns the first entry's contents.
    """
    with zipfile.ZipFile(io.BytesIO(blob)) as archive:
        return archive.read(archive.namelist()[0])


#: The order GenICam's schema requires a node's children to appear in. It
#: declares them as a sequence, not a set, so a strict parser stops at the
#: first element out of place -- and stopping means the whole document is
#: rejected and the camera cannot be opened at all.
#:
#: Aravis does not check, which is what makes this worth validating here:
#: a document that works perfectly against every client on the bench can be
#: unloadable by the vendor client it was written for. VimbaX answered one
#: with InternalFault and no further detail; the reason was only visible in
#: its own log, as "expected element not encountered" at a line number.
#:
#: Only the elements this generator emits are listed. Anything absent from
#: the list is ignored rather than rejected, so adding an element means
#: adding it here in the right place.
CHILD_ORDER = {
    "IntReg": ("ToolTip", "Description", "DisplayName", "Visibility",
               "Address", "Length", "AccessMode", "pPort", "Cachable",
               "PollingTime", "pInvalidator", "Sign", "Endianess", "Unit",
               "Representation"),
    "FloatReg": ("ToolTip", "Description", "DisplayName", "Visibility",
                 "Address", "Length", "AccessMode", "pPort", "Cachable",
                 "PollingTime", "pInvalidator", "Endianess", "Unit",
                 "Representation"),
    "MaskedIntReg": ("ToolTip", "Description", "DisplayName", "Visibility",
                     "Address", "Length", "AccessMode", "pPort", "Cachable",
                     "PollingTime", "pInvalidator", "LSB", "MSB", "Sign",
                     "Endianess", "Unit", "Representation"),
    "StringReg": ("ToolTip", "Description", "DisplayName", "Visibility",
                  "Address", "Length", "AccessMode", "pPort", "Cachable",
                  "PollingTime", "pInvalidator"),
    "Integer": ("ToolTip", "Description", "DisplayName", "Visibility",
                "Value", "pValue", "Min", "pMin", "Max", "pMax", "Inc",
                "pInc", "Unit", "Representation"),
    "Float": ("ToolTip", "Description", "DisplayName", "Visibility",
              "Value", "pValue", "Min", "pMin", "Max", "pMax", "Unit",
              "Representation", "DisplayNotation", "DisplayPrecision"),
    "Enumeration": ("ToolTip", "Description", "DisplayName", "Visibility",
                    "EnumEntry", "pValue"),
    "EnumEntry": ("ToolTip", "Description", "DisplayName", "Visibility",
                  "Value", "NumericValue"),
    "Command": ("ToolTip", "Description", "DisplayName", "Visibility",
                "pValue", "CommandValue"),
    "Category": ("ToolTip", "Description", "DisplayName", "Visibility",
                 "pFeature"),
}


def _check_child_order(element, tag, problems):
    order = CHILD_ORDER.get(element.tag[len(tag):])
    if order is None:
        return
    seen = -1
    for child in element:
        if not child.tag.startswith(tag):
            continue
        local = child.tag[len(tag):]
        if local not in order:
            continue
        position = order.index(local)
        if position < seen:
            problems.append(
                "%s %r puts <%s> after <%s>; the schema wants it before"
                % (element.tag[len(tag):], element.get("Name"), local,
                   order[seen]))
            return
        seen = max(seen, position)


def validate_xml(xml_bytes, feature_set):
    """
    Structural checks that would otherwise only show up as a client that
    silently refuses to stream. Called from the tests, and cheap enough to
    call at startup.
    """
    root = ElementTree.fromstring(xml_bytes)
    problems = []

    # Transport registers this device publishes on purpose. Everything else
    # carrying a Gev name is a collision with the nodes the client injects.
    published = {f.name for f in feature_set.features if f.transport}
    published |= {f.name + "Reg" for f in feature_set.features if f.transport}

    tag = "{%s}" % SCHEMA_NS
    if not root.tag.endswith("RegisterDescription"):
        problems.append("root element is %r" % root.tag)

    names = set()
    for element in root.iter():
        name = element.get("Name")
        if name is None:
            continue
        names.add(name)
        if ((name.startswith("Gev") or name.startswith("ArvGev"))
                and name not in published):
            problems.append("%r collides with an injected transport node" % name)

    if "Device" not in names:
        problems.append("missing <Port Name=\"Device\"/>")

    # A pInvalidator or pMax naming something that does not exist gives a
    # document the client rejects at parse time, which surfaces as a camera
    # that cannot be opened at all rather than as one feature misbehaving.
    # Catching it here turns a mistyped dependency into a startup error that
    # names the feature.
    for feature in feature_set.features:
        for name in feature.invalidated_by:
            if name not in feature_set.by_name:
                problems.append("%s is invalidated by %r, which is not a "
                                "declared feature" % (feature.name, name))
        for which in ("p_min", "p_max"):
            pointer = getattr(feature, which, None)
            if pointer is None:
                continue
            target = feature_set.by_name.get(pointer)
            if target is None:
                problems.append("%s.%s names %r, which is not a declared "
                                "feature" % (feature.name, which, pointer))
            elif not isinstance(target, (IntFeature, FloatFeature)):
                # <pMax> has to resolve to something with a value. Pointing
                # it at a Command or an Enumeration parses and then fails in
                # the client's node graph, well away from the cause.
                problems.append("%s.%s names %r, which is a %s and carries "
                                "no numeric value"
                                % (feature.name, which, pointer,
                                   type(target).__name__))

    for element in root.iter():
        _check_child_order(element, tag, problems)

    # Every address in the document must match what the allocator assigned.
    for element in root.iter(tag + "IntReg"):
        _check_address(element, tag, feature_set, problems)
    for element in root.iter(tag + "FloatReg"):
        _check_address(element, tag, feature_set, problems)
    for element in root.iter(tag + "StringReg"):
        _check_address(element, tag, feature_set, problems)
    for element in root.iter(tag + "MaskedIntReg"):
        _check_address(element, tag, feature_set, problems)

    return problems


def _check_address(element, tag, feature_set, problems):
    name = element.get("Name")
    address_element = element.find(tag + "Address")
    if address_element is None:
        problems.append("%s has no <Address>" % name)
        return
    address = int(address_element.text, 0)
    feature = feature_set.lookup_address(address)
    # A transport register belongs in the bootstrap page, at the address the
    # standard gives it, and is the one kind of node that is meant to sit
    # outside the arena.
    if not (feature_set.arena_start <= address < feature_set.arena_end):
        if feature is None or not feature.transport:
            problems.append("%s address 0x%x is outside the feature arena"
                            % (name, address))
        return
    if feature is None:
        problems.append("%s address 0x%x matches no allocated feature"
                        % (name, address))
