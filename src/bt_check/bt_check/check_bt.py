import xml.etree.ElementTree as ET
import xml.dom.minidom

class BTChecker:
    def __init__(self, _valid_actions, _valid_control_nodes):
        self.feedback = ""
        self.valid_actions = _valid_actions
        self.valid_control_nodes = _valid_control_nodes
        self.root = None
    
    def is_float(self, value):
        try:
            float(value)
            return True
        except ValueError:
            return False
    
    def is_valid_node(self, element):
        valid = True
        if element.tag in self.valid_actions:
            required_attrs = self.valid_actions[element.tag]
            # check all attributes in element is required_attrs
            for attr in element.attrib:
                if attr not in required_attrs:
                    string = f"Invalid attribute '{attr}' in {element.tag} "
                    self.feedback += string + "\n"
                    valid = False
                if not self.is_float(element.get(attr)):
                    string = f"Invalid value '{element.get(attr)}' for attribute '{attr}' in {element.tag} "
                    self.feedback += string + "\n"
                    valid = False
        elif element.tag in self.valid_control_nodes:
            required_attrs = self.valid_control_nodes[element.tag]
            for attr in required_attrs:
                if element.get(attr) is None:
                    string = f"Missing attribute '{attr}' in {element.tag} "
                    self.feedback += string + "\n"
                    valid = False
        else:
            string = f"Invalid element found: {ET.tostring(element, 'utf-8').decode('utf-8')}"
            self.feedback += string + "\n"
            valid = False
        return valid
    
    def check_all_nodes(self, element):
        valid = True
        if not self.is_valid_node(element):
            valid = False
        for child in element:
            if not self.check_all_nodes(child):
                valid = False
        return valid
    
    def check_behavior_tree(self, xml_content):
        try:
            self.root = ET.fromstring(xml_content)
            while self.is_valid_node(self.root)==False and len(self.root)==1 and self.is_valid_node(self.root[0])==False and len(self.root[0])==1:
                self.root = self.root[0]
            self.root.tag = "ROOOOOT"
            if self.check_all_nodes(self.root):
                return True
            else:
                string = "The behavior tree contains invalid elements."
                self.feedback += string + "\n"
                return False
        except ET.ParseError as e:
            string = f"Error parsing XML: {e}"
            self.feedback += string + "\n"
            return False
    
    def generate_xml_tree(self):
        total_string = ""
        for child in self.root:
            total_string += ET.tostring(child, 'utf-8').decode('utf-8')
        return total_string
