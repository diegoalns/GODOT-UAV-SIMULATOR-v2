"""
Streamlit Interactive Map Viewer for 3D Lattice Graph
Displays Layer 0 nodes from the regular_lattice_graph.pkl file on an interactive map.
"""

import streamlit as st
import pickle
import pandas as pd
import folium
from streamlit_folium import st_folium
import os

# --- Configuration ---
PICKLE_FILENAME = "regular_lattice_graph.pkl"

# --- Load Data ---
@st.cache_data
def load_graph_data(pickle_path):
    """Load the graph data from pickle file."""
    try:
        with open(pickle_path, 'rb') as f:
            data = pickle.load(f)
        return data['graph'], data['metadata']
    except FileNotFoundError:
        st.error(f"Error: Pickle file '{pickle_path}' not found!")
        st.info("Please run Regular_3D_Lattice_Graph.py first to generate the graph.")
        return None, None
    except Exception as e:
        st.error(f"Error loading pickle file: {e}")
        return None, None

def extract_layer_nodes(G, layer=0):
    """Extract nodes from a specific layer."""
    layer_nodes = []
    
    for node_id, node_data in G.nodes(data=True):
        if node_data['layer'] == layer:
            layer_nodes.append({
                'node_id': node_id,
                'lat': node_data['lat'],
                'lon': node_data['lon'],
                'grid_x': node_data['grid_x'],
                'grid_y': node_data['grid_y'],
                'altitude_ft': node_data['altitude_ft'],
                'available': node_data['available'],
                'faa_ceiling_ft': node_data.get('faa_ceiling_ft', 0),
                'max_obstacle_ft': node_data.get('max_obstacle_ft', 0),
                'unavailable_reason': node_data.get('unavailable_reason', 'N/A')
            })
    
    return pd.DataFrame(layer_nodes)

def create_map(nodes_df, center_lat=None, center_lon=None):
    """Create a Folium map with nodes."""
    
    # Calculate center if not provided
    if center_lat is None:
        center_lat = nodes_df['lat'].mean()
    if center_lon is None:
        center_lon = nodes_df['lon'].mean()
    
    # Create map
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=13,
        tiles='OpenStreetMap'
    )
    
    # Add tile layer options
    folium.TileLayer('CartoDB positron').add_to(m)
    folium.TileLayer('CartoDB dark_matter').add_to(m)
    
    # Add nodes as markers
    for idx, row in nodes_df.iterrows():
        # Color based on availability
        if row['available']:
            color = 'green'
            icon = 'ok-sign'
        else:
            color = 'red'
            icon = 'remove-sign'
        
        # Create popup with node information
        popup_html = f"""
        <div style="font-family: Arial; font-size: 12px;">
            <b>Node ID:</b> {row['node_id']}<br>
            <b>Grid Position:</b> X{row['grid_x']}, Y{row['grid_y']}<br>
            <b>Coordinates:</b> ({row['lat']:.6f}, {row['lon']:.6f})<br>
            <b>Altitude:</b> {row['altitude_ft']:.0f} ft<br>
            <b>Available:</b> {row['available']}<br>
            <b>FAA Ceiling:</b> {row['faa_ceiling_ft']:.0f} ft<br>
            <b>Max Obstacle:</b> {row['max_obstacle_ft']:.0f} ft<br>
            <b>Status:</b> {row['unavailable_reason']}
        </div>
        """
        
        # Add marker
        folium.Marker(
            location=[row['lat'], row['lon']],
            popup=folium.Popup(popup_html, max_width=300),
            tooltip=f"{row['node_id']}<br>Available: {row['available']}",
            icon=folium.Icon(color=color, icon=icon, prefix='glyphicon')
        ).add_to(m)
    
    # Add layer control
    folium.LayerControl().add_to(m)
    
    return m

# --- Main App ---
def main():
    st.set_page_config(
        page_title="3D Lattice Graph Map Viewer",
        page_icon="🗺️",
        layout="wide"
    )
    
    st.title("🗺️ 3D Lattice Graph - Interactive Map Viewer")
    st.markdown("### New York Area - Layer 0 (Ground Level)")
    
    # Check if pickle file exists
    if not os.path.exists(PICKLE_FILENAME):
        st.error(f"❌ Pickle file '{PICKLE_FILENAME}' not found!")
        st.info("Please run `Regular_3D_Lattice_Graph.py` first to generate the graph data.")
        return
    
    # Load graph data
    with st.spinner("Loading graph data..."):
        G, metadata = load_graph_data(PICKLE_FILENAME)
    
    if G is None:
        return
    
    # Display metadata
    with st.sidebar:
        st.header("Graph Information")
        st.markdown(f"**Created:** {metadata.get('created_at', 'N/A')}")
        st.markdown(f"**Grid Dimensions:** {metadata.get('grid_dimensions', 'N/A')}")
        st.markdown(f"**Total Nodes:** {metadata.get('num_nodes', 0):,}")
        st.markdown(f"**Total Edges:** {metadata.get('num_edges', 0):,}")
        st.markdown(f"**Horizontal Spacing:** {metadata.get('horizontal_spacing_m', 0)} m")
        st.markdown(f"**Vertical Spacing:** {metadata.get('vertical_spacing_ft', 0)} ft")
        
        st.divider()
        
        # Layer selector
        st.header("Layer Selection")
        selected_layer = st.selectbox(
            "Select Layer",
            options=list(range(metadata.get('num_layers', 5))),
            format_func=lambda x: f"Layer {x} ({metadata.get('layer_altitudes_ft', [])[x] if x < len(metadata.get('layer_altitudes_ft', [])) else 0} ft)",
            index=0
        )
        
        st.divider()
        
        # Display options
        st.header("Display Options")
        show_available_only = st.checkbox("Show Available Nodes Only", value=False)
        show_unavailable_only = st.checkbox("Show Unavailable Nodes Only", value=False)
    
    # Extract layer nodes
    with st.spinner(f"Loading Layer {selected_layer} nodes..."):
        nodes_df = extract_layer_nodes(G, layer=selected_layer)
    
    # Filter nodes based on display options
    filtered_df = nodes_df.copy()
    if show_available_only:
        filtered_df = filtered_df[filtered_df['available'] == True]
    elif show_unavailable_only:
        filtered_df = filtered_df[filtered_df['available'] == False]
    
    # Display statistics
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        st.metric("Total Nodes", len(nodes_df))
    
    with col2:
        available_count = nodes_df['available'].sum()
        st.metric("Available Nodes", available_count)
    
    with col3:
        unavailable_count = len(nodes_df) - available_count
        st.metric("Unavailable Nodes", unavailable_count)
    
    with col4:
        availability_pct = (available_count / len(nodes_df) * 100) if len(nodes_df) > 0 else 0
        st.metric("Availability", f"{availability_pct:.1f}%")
    
    # Create and display map
    if len(filtered_df) > 0:
        st.markdown(f"### Displaying {len(filtered_df)} nodes on map")
        
        with st.spinner("Generating map..."):
            map_obj = create_map(filtered_df)
        
        # Display map
        st_folium(map_obj, width=1400, height=700)
        
        # Display node data table
        with st.expander("📊 View Node Data Table"):
            st.dataframe(
                filtered_df,
                use_container_width=True,
                column_config={
                    "lat": st.column_config.NumberColumn("Latitude", format="%.6f"),
                    "lon": st.column_config.NumberColumn("Longitude", format="%.6f"),
                    "altitude_ft": st.column_config.NumberColumn("Altitude (ft)", format="%.0f"),
                    "faa_ceiling_ft": st.column_config.NumberColumn("FAA Ceiling (ft)", format="%.0f"),
                    "max_obstacle_ft": st.column_config.NumberColumn("Max Obstacle (ft)", format="%.0f"),
                    "available": st.column_config.CheckboxColumn("Available"),
                }
            )
        
        # Download button
        csv_data = filtered_df.to_csv(index=False)
        st.download_button(
            label="📥 Download Node Data as CSV",
            data=csv_data,
            file_name=f"layer_{selected_layer}_nodes.csv",
            mime="text/csv"
        )
    else:
        st.warning("No nodes to display with current filters.")
    
    # Footer
    st.divider()
    st.markdown("""
    **Legend:**
    - 🟢 Green markers: Available nodes (meet FAA and obstacle clearance requirements)
    - 🔴 Red markers: Unavailable nodes (violate FAA ceiling or obstacle clearance)
    - Click on markers for detailed information
    - Use the layer control (top-right of map) to switch between map styles
    """)

if __name__ == "__main__":
    main()
