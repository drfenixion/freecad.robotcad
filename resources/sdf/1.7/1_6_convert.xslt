<!-- This stylesheet is provided for reference. It is not currently used by libsdformat. -->
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0">

  <xsl:output method="xml" indent="yes" />
  <xsl:decimal-format decimal-separator="." grouping-separator="," />

  <!-- Identity template, provides default behavior that copies all content. -->
  <xsl:template match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

  <!-- sdf version attribute 1.6 -> 1.7 -->
  <xsl:template match="/sdf[@version='1.6']">
    <xsl:element name="sdf">
      <xsl:attribute name="version">
        <xsl:value-of select="1.7"/>
      </xsl:attribute>
      <xsl:apply-templates select="node()"/>
    </xsl:element>
  </xsl:template>

  <xsl:template match="/sdf[@version='1.6']//pose/@frame">
    <xsl:attribute name="relative_to">
      <xsl:value-of select="."/>
    </xsl:attribute>
  </xsl:template>

  <xsl:template match="/sdf[@version='1.6']//joint/axis|/sdf[@version='1.6']//joint/axis2">
    <xsl:copy>
      <xsl:apply-templates select="@*"/>
      <!-- the text element puts the xyz element on the next line with fixed indentation -->
      <xsl:text>
        </xsl:text>
      <xsl:element name="xyz">
        <xsl:apply-templates select="xyz/@*"/>
        <!-- case-insensitive test for use_parent_model_frame = "true" or "1" -->
        <xsl:if test="translate(use_parent_model_frame, 'TRUE', 'true')='true' or use_parent_model_frame='1'">
          <!-- if so, add expressed_in="__model__" attribute -->
          <xsl:attribute name="expressed_in">
            <xsl:text>__model__</xsl:text>
          </xsl:attribute>
        </xsl:if>
        <xsl:apply-templates select="xyz/node()"/>
      </xsl:element>
      <xsl:apply-templates select="node()"/>
    </xsl:copy>
  </xsl:template>
  <xsl:template match="/sdf[@version='1.6']//joint/axis/xyz" />
  <xsl:template match="/sdf[@version='1.6']//joint/axis/use_parent_model_frame" />
  <xsl:template match="/sdf[@version='1.6']//joint/axis2/xyz" />
  <xsl:template match="/sdf[@version='1.6']//joint/axis2/use_parent_model_frame" />

</xsl:stylesheet>
